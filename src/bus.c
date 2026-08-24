/*
 * Memory bus adapter — routes recompiled code memory accesses to
 * LakeSnes's real SNES hardware (PPU, APU, DMA, cart, WRAM).
 *
 * Your recompiled code calls bus_read8/bus_write8 with a 24-bit
 * SNES address (bank:addr). We convert that to a flat 24-bit address
 * and hand it to LakeSnes's snes_read/snes_write, which routes it
 * to the correct hardware component.
 */

#include "snesrecomp/bus.h"
#include "snesrecomp/cpu.h"
#include "snes.h"
#include "cart.h"
#include "apu.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* Access to the LakeSnes instance (owned by snesrecomp.c) */
extern Snes *snesrecomp_get_snes(void);

static int s_phase_mode = -1;
static bool s_phase_active = false;
static uint32_t s_phase_frame_at_begin = 0;
static bool s_phase_vblank_at_begin = false;
static uint32_t s_phase_pc = 0;

static bool phase_mode(void) {
    if (s_phase_mode < 0) {
        const char *e = getenv("SMK_RECOMP_BUSPHASE");
        s_phase_mode = e ? atoi(e) != 0 : 0;
    }
    return s_phase_mode != 0;
}

static void phase_advance(int cycles) {
    Snes *snes = snesrecomp_get_snes();
    if (!snes || cycles <= 0) return;
    dma_handleDma(snes->dma, cycles);
    snes_runCycles(snes, cycles);
}

/* Cycle-accurate model: a recompiled function advances the master clock by its
 * real per-instruction cost (autogen emits recomp_tick() calls) so the PPU/APU/
 * auto-joypad stay in phase with genuine timing — fixing the instant-execution
 * skew that makes timing-sensitive / transition functions diverge. Mirrors
 * snes_cpuRead's per-access stepping (dma_handleDma + snes_runCycles). Runtime-
 * gated by SMK_RECOMP_CYCLEACCURATE so the same binary can A/B test instant vs
 * cycle-accurate. */
void recomp_tick(int cycles) {
    static int on = -1;
    if (on < 0) {
        const char *e = getenv("SMK_RECOMP_CYCLEACCURATE");
        on = e ? atoi(e) : 1;   /* default ON: more correct (fixes timing-sensitive
                                 * funcs) and byte-identical for steady-state ones */
    }
    if (!on || cycles <= 0) return;
    Snes *snes = snesrecomp_get_snes();
    if (!snes) return;
    dma_handleDma(snes->dma, cycles);
    snes_runCycles(snes, cycles);
}

void recomp_phase_begin_checked(int aggregate_cycles, uint8_t bank,
                                uint16_t pc, int instruction_bytes,
                                int check_after_fetches) {
    if (!phase_mode()) {
        s_phase_active = false;
        recomp_tick(aggregate_cycles);
        return;
    }
    Snes *snes = snesrecomp_get_snes();
    if (!snes) return;
    s_phase_active = true;
    s_phase_frame_at_begin = snes->frames;
    s_phase_vblank_at_begin = snes->inVblank;
    s_phase_pc = ((uint32_t)bank << 16) | pc;
    for (int i = 0; i < instruction_bytes; i++) {
        (void)snes_cpuRead(snes, ((uint32_t)bank << 16) | (uint16_t)(pc + i));
        if (i + 1 == check_after_fetches) recomp_phase_check_int();
    }
}

void recomp_phase_begin(int aggregate_cycles, uint8_t bank, uint16_t pc,
                        int instruction_bytes) {
    recomp_phase_begin_checked(aggregate_cycles, bank, pc,
                               instruction_bytes, -1);
}

void recomp_phase_idle(int master_cycles) {
    if (!phase_mode()) return;
    while (master_cycles >= 6) {
        Snes *snes = snesrecomp_get_snes();
        if (snes) snes_cpuIdle(snes, false);
        master_cycles -= 6;
    }
    if (master_cycles > 0) phase_advance(master_cycles);
}

void recomp_phase_end(int internal_cycles, int synthetic_stack_accesses) {
    if (!phase_mode()) return;
    /* Synthetic accesses account for RTS/RTL stack pulls performed later by
     * the interception hook without touching live stack bytes a second time. */
    for (int i = 0; i < synthetic_stack_accesses; i++) phase_advance(8);
    recomp_phase_idle(internal_cycles);
    s_phase_active = false;
}

static void phase_stack_dec(void) {
    g_cpu.S--;
    if (g_cpu.flag_E) g_cpu.S = (uint16_t)((g_cpu.S & 0x00FF) | 0x0100);
}

static void phase_stack_inc(void) {
    g_cpu.S++;
    if (g_cpu.flag_E) g_cpu.S = (uint16_t)((g_cpu.S & 0x00FF) | 0x0100);
}

void recomp_stack_push8(uint8_t value) {
    bus_wram_write8(g_cpu.S, value);
    phase_stack_dec();
}

uint8_t recomp_stack_pull8(void) {
    phase_stack_inc();
    return bus_wram_read8(g_cpu.S);
}

void recomp_stack_push16(uint16_t value, bool check_between) {
    recomp_stack_push8((uint8_t)(value >> 8));
    if (check_between) recomp_phase_check_int();
    recomp_stack_push8((uint8_t)value);
}

uint16_t recomp_stack_pull16(bool check_between) {
    uint8_t low = recomp_stack_pull8();
    if (check_between) recomp_phase_check_int();
    return (uint16_t)(low | ((uint16_t)recomp_stack_pull8() << 8));
}

recomp_call_frame_t recomp_phase_call_enter(uint16_t return_pc,
                                             uint8_t return_bank,
                                             uint8_t target_bank,
                                             bool is_long,
                                             bool check_return_push) {
    recomp_call_frame_t frame = {
        g_cpu.S, return_bank, 0, is_long
    };
    if (!phase_mode()) {
        if (is_long) g_cpu.PB = target_bank;
        return frame;
    }
    /* JSR: operand fetches, idle, push return high then low.
     * JSL additionally pushes PB. The current prototype fetches the long-bank
     * operand with the other opcode bytes, so PB is materialized here. */
    if (is_long) {
        recomp_stack_push8(return_bank);
    }
    {
        Snes *snes = snesrecomp_get_snes();
        if (snes) snes_cpuIdle(snes, false);
    }
    recomp_stack_push16(return_pc, check_return_push);
    if (is_long) g_cpu.PB = target_bank;
    s_phase_active = false;
    frame.frame_bytes = is_long ? 3 : 2;
    return frame;
}

void recomp_phase_return(bool is_long) {
    if (!phase_mode()) return;
    /* LakeSnes: RTS = idle, idle, pull lo/hi, checkInt, idle.
     * RTL = idle, idle, pull lo/hi, checkInt, pull PB. The interception hook
     * performs the architectural pulls, so these are timing-only accesses. */
    recomp_phase_idle(12);
    phase_advance(16);
    recomp_phase_check_int();
    if (is_long) phase_advance(8);
    else recomp_phase_idle(6);
    s_phase_active = false;
}

void recomp_phase_call_leave(recomp_call_frame_t frame,
                             bool frame_consumed) {
    if (frame_consumed) return;
    uint8_t frame_bytes = frame.frame_bytes;
    while (frame_bytes-- > 0) phase_stack_inc();
    if (frame.is_long) {
        g_cpu.PB = frame.frame_bytes
            ? bus_wram_read8(g_cpu.S)
            : frame.return_bank;
    }
}

void recomp_phase_check_int(void) {
    if (!phase_mode()) return;
    Snes *snes = snesrecomp_get_snes();
    if (!snes || !snes->cpu) return;
    bool pending = snes->cpu->nmiWanted ||
                   (snes->cpu->irqWanted && !g_cpu.flag_I);
    snes->cpu->intWanted = pending;
}

bool recomp_phase_interrupt_pending(void) {
    if (!phase_mode()) return false;
    Snes *snes = snesrecomp_get_snes();
    if (!snes || !snes->cpu) return false;
    bool pending = snes->cpu->intWanted;
    bool frame_boundary = snes->frames != s_phase_frame_at_begin;
    bool vblank_boundary = !s_phase_vblank_at_begin && snes->inVblank;
    if (getenv("SMK_RECOMP_PHASE_DEBUG") &&
        (frame_boundary || vblank_boundary ||
         (s_phase_pc == 0x81F687 && snes->vPos >= 220))) {
        fprintf(stderr, "[PHASE] pc=%06X frame=%u v=%u h=%u vb=%d start_vb=%d boundary=%d\n",
                s_phase_pc, snes->frames, snes->vPos, snes->hPos,
                snes->inVblank, s_phase_vblank_at_begin,
                frame_boundary || vblank_boundary);
    }
    static int yield_on = -1;
    if (yield_on < 0) {
        const char *e = getenv("SMK_RECOMP_PHASE_YIELD");
        yield_on = e ? atoi(e) != 0 : 0;
    }
    /* snes_runFrame can only observe completion after the native hook returns.
     * Never let one C body run instructions belonging to the next video frame. */
    return frame_boundary || vblank_boundary || (yield_on && pending);
}

uint8_t bus_read8(uint8_t bank, uint16_t addr) {
    Snes *snes = snesrecomp_get_snes();
    if (!snes) return 0;
    uint32_t flat = ((uint32_t)bank << 16) | addr;
    return s_phase_active ? snes_cpuRead(snes, flat) : snes_read(snes, flat);
}

uint8_t bus_read8_checked(uint8_t bank, uint16_t addr) {
    recomp_phase_check_int();
    return bus_read8(bank, addr);
}

void bus_write8(uint8_t bank, uint16_t addr, uint8_t val) {
    Snes *snes = snesrecomp_get_snes();
    if (!snes) return;
    uint32_t flat = ((uint32_t)bank << 16) | addr;
    if (s_phase_active) snes_cpuWrite(snes, flat, val);
    else snes_write(snes, flat, val);

    /* Diagnostic: count/trace ALL CGRAM-data writes (CPU interp + recompiled C
     * both pass through here), to find non-interpreter CGRAM writers. */
    if (addr == 0x2122 && getenv("SMK_CG_DEBUG")) {
        static int n = 0;
        fprintf(stderr, "[CG] #%d $2122 <- %02X (bank=%02X)\n", n++, val, bank);
    }

    /* Diagnostic: trace the DMA channel-0 SETUP registers + trigger with the
     * CPU PC, to see whether the game performs per-DMA setup ($4302 srcL,
     * $4303 srcH, $4304 srcBank, $4305/6 size, $2121 CGADD) before each $420B,
     * or fires $420B with stale channel regs (race-start palette bug). */
    if (getenv("SMK_DMASETUP_DEBUG")) {
        uint16_t pc = (snes->cpu ? snes->cpu->pc : 0);
        uint8_t  k  = (snes->cpu ? snes->cpu->k : 0);
        if (addr == 0x2121 || addr == 0x4300 || addr == 0x4301 ||
            addr == 0x4302 || addr == 0x4303 ||
            addr == 0x4304 || addr == 0x4305 || addr == 0x4306 ||
            addr == 0x420B)
            fprintf(stderr, "[SET] $%04X <- %02X  pc=%02X:%04X\n",
                    addr, val, k, pc);
    }

    /* DMA fix: LakeSnes uses a 2-step state machine for DMA:
     *   Step 1: dmaState 1→2 (arm)
     *   Step 2: dmaState 2→0 (execute transfer)
     * Normally each step happens on successive CPU access cycles.
     * Since recompiled code bypasses the CPU, we call handleDma twice
     * to complete both steps immediately. */
    if (!s_phase_active && (addr == 0x420B || addr == 0x420C)) {
        /* Diagnostic: log GPDMA channel params (esp. CGRAM-targeting) so the
         * race-start palette-corrupting transfer can be identified. */
        if (addr == 0x420B && getenv("SMK_DMA_DEBUG")) {
            for (int c = 0; c < 8; c++) {
                if (!(val & (1 << c))) continue;
                fprintf(stderr, "[DMA] ch%d $420B=%02X -> $21%02X  src=%02X:%04X size=%04X mode=%d\n",
                        c, val, snes->dma->channel[c].bAdr,
                        snes->dma->channel[c].aBank, snes->dma->channel[c].aAdr,
                        snes->dma->channel[c].size, snes->dma->channel[c].mode);
            }
        }
        dma_handleDma(snes->dma, 8);  /* state 1→2 (arm) */
        dma_handleDma(snes->dma, 8);  /* state 2→0 (execute) */
    }
}

void bus_write8_checked(uint8_t bank, uint16_t addr, uint8_t val) {
    recomp_phase_check_int();
    bus_write8(bank, addr, val);
}

uint16_t bus_read16(uint8_t bank, uint16_t addr) {
    uint8_t lo = bus_read8(bank, addr);
    uint16_t next_addr = addr + 1;
    uint8_t next_bank = bank;
    if (next_addr == 0x0000) {
        next_bank++;
    }
    uint8_t hi = bus_read8(next_bank, next_addr);
    return (uint16_t)(lo | (hi << 8));
}

uint16_t bus_read16_checked(uint8_t bank, uint16_t addr) {
    uint8_t lo = bus_read8(bank, addr);
    uint16_t next_addr = (uint16_t)(addr + 1);
    uint8_t next_bank = bank;
    if (next_addr == 0x0000) next_bank++;
    recomp_phase_check_int();
    return (uint16_t)(lo | ((uint16_t)bus_read8(next_bank, next_addr) << 8));
}

void bus_write16(uint8_t bank, uint16_t addr, uint16_t val) {
    bus_write8(bank, addr, (uint8_t)(val & 0xFF));
    uint16_t next_addr = addr + 1;
    uint8_t next_bank = bank;
    if (next_addr == 0x0000) {
        next_bank++;
    }
    bus_write8(next_bank, next_addr, (uint8_t)(val >> 8));
}

void bus_write16_checked(uint8_t bank, uint16_t addr, uint16_t val) {
    bus_write8(bank, addr, (uint8_t)val);
    uint16_t next_addr = (uint16_t)(addr + 1);
    uint8_t next_bank = bank;
    if (next_addr == 0x0000) next_bank++;
    recomp_phase_check_int();
    bus_write8(next_bank, next_addr, (uint8_t)(val >> 8));
}

void bus_write16_reversed_checked(uint8_t bank, uint16_t addr, uint16_t val) {
    uint16_t next_addr = (uint16_t)(addr + 1);
    uint8_t next_bank = bank;
    if (next_addr == 0x0000) next_bank++;
    bus_write8(next_bank, next_addr, (uint8_t)(val >> 8));
    recomp_phase_check_int();
    bus_write8(bank, addr, (uint8_t)val);
}

/* --- Direct WRAM access (bypass bus for speed) --- */

uint8_t bus_wram_read8(uint32_t offset) {
    Snes *snes = snesrecomp_get_snes();
    if (!snes) return 0;
    if (s_phase_active) phase_advance(8);
    return snes->ram[offset & 0x1FFFF];
}

void bus_wram_write8(uint32_t offset, uint8_t val) {
    Snes *snes = snesrecomp_get_snes();
    if (!snes) return;
    if (s_phase_active) phase_advance(8);
    snes->ram[offset & 0x1FFFF] = val;
}

uint16_t bus_wram_read16(uint32_t offset) {
    uint8_t lo = bus_wram_read8(offset);
    uint8_t hi = bus_wram_read8(offset + 1);
    return (uint16_t)(lo | (hi << 8));
}

void bus_wram_write16(uint32_t offset, uint16_t val) {
    bus_wram_write8(offset, (uint8_t)(val & 0xFF));
    bus_wram_write8(offset + 1, (uint8_t)(val >> 8));
}

uint8_t *bus_get_wram(void) {
    Snes *snes = snesrecomp_get_snes();
    if (!snes) return NULL;
    return snes->ram;
}

const uint8_t *bus_get_rom(uint32_t *size_out) {
    Snes *snes = snesrecomp_get_snes();
    if (!snes || !snes->cart) return NULL;
    if (size_out) *size_out = snes->cart->romSize;
    return snes->cart->rom;
}

void bus_apu_write_ram(uint16_t spc_addr, uint8_t val) {
    Snes *snes = snesrecomp_get_snes();
    if (!snes || !snes->apu) return;
    snes->apu->ram[spc_addr] = val;
}

uint8_t bus_apu_read_ram(uint16_t spc_addr) {
    Snes *snes = snesrecomp_get_snes();
    if (!snes || !snes->apu) return 0;
    return snes->apu->ram[spc_addr];
}

void bus_run_cycles(int master_cycles) {
    Snes *snes = snesrecomp_get_snes();
    if (!snes || !snes->apu) return;
    /* Convert master cycles to APU cycles.
     * NTSC: APU runs at ~1.024 MHz, master clock ~21.477 MHz.
     * Ratio is roughly 1 APU cycle per 21 master cycles,
     * but LakeSnes uses fractional accumulation internally.
     * We directly run the APU for an equivalent number of cycles. */
    int apu_cycles = master_cycles;  /* 1:1 works — apu_runCycles counts SPC clocks */
    apu_runCycles(snes->apu, apu_cycles);
}

