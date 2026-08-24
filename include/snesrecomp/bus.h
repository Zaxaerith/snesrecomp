#ifndef SNESRECOMP_BUS_H
#define SNESRECOMP_BUS_H

/*
 * Memory bus — the bridge between your recompiled code and real SNES hardware.
 *
 * When your recompiled function does:
 *   STA $2118    (write to PPU VRAM data port)
 *
 * It becomes:
 *   bus_write8(g_cpu.DB, 0x2118, CPU_A8());
 *
 * That write goes through LakeSnes's full memory bus to the real PPU,
 * which updates VRAM exactly like the original hardware would.
 *
 * Same for reads — LDA $4210 (read NMI status) becomes:
 *   CPU_SET_A8(bus_read8(0x00, 0x4210));
 *
 * You get cycle-accurate hardware behavior for free.
 */

#include <stdint.h>
#include <stdbool.h>

/* 24-bit address space reads/writes (bank:addr) */
uint8_t  bus_read8(uint8_t bank, uint16_t addr);
void     bus_write8(uint8_t bank, uint16_t addr, uint8_t val);
uint16_t bus_read16(uint8_t bank, uint16_t addr);
void     bus_write16(uint8_t bank, uint16_t addr, uint16_t val);

/* Cycle-accurate model: advance the master clock by `cycles` (no-op unless
 * SMK_RECOMP_CYCLEACCURATE is set). Autogen emits one call per recompiled
 * instruction with that instruction's real master-cycle cost. */
void     recomp_tick(int cycles);

/* Experimental bus-phase timing for generated instructions. When
 * SMK_RECOMP_BUSPHASE=1, begin performs timed opcode/operand fetches, normal
 * bus helpers perform timed data accesses, and end advances internal/stack
 * phases. Without the flag, begin is exactly the legacy aggregate tick. */
void     recomp_phase_begin(int aggregate_cycles, uint8_t bank, uint16_t pc,
                            int instruction_bytes);
void     recomp_phase_begin_checked(int aggregate_cycles, uint8_t bank,
                                    uint16_t pc, int instruction_bytes,
                                    int check_after_fetches);
void     recomp_phase_idle(int master_cycles);
void     recomp_phase_check_int(void);
void     recomp_phase_end(int internal_cycles, int synthetic_stack_accesses);
void     recomp_phase_return(bool is_long);
bool     recomp_phase_interrupt_pending(void);
typedef struct recomp_call_frame {
    uint16_t caller_sp;
    uint8_t  return_bank;
    uint8_t  frame_bytes;
    bool     is_long;
} recomp_call_frame_t;
/* Enter always applies JSL's target PB. Bus-phase mode additionally
 * materializes the native JSR/JSL return frame and records its original SP.
 * A redirected call keeps that frame live so ROM execution can resume inside
 * the callee. On a normal return, leave either emulates the native pull or does
 * nothing when an interpreter fallback already consumed the physical frame. */
recomp_call_frame_t recomp_phase_call_enter(uint16_t return_pc,
                                             uint8_t return_bank,
                                             uint8_t target_bank,
                                             bool is_long,
                                             bool check_return_push);
/* JSR (abs,X) has a unique sequence: fetch opcode+operand-low, push the
 * return word, fetch operand-high, idle, then read/check/read the table. This
 * helper performs the middle segment and deliberately leaves the phase open
 * for the generated table reads. */
recomp_call_frame_t recomp_phase_call_enter_indirect(uint16_t return_pc,
                                                      uint8_t return_bank);
void     recomp_phase_call_leave(recomp_call_frame_t frame,
                                 bool frame_consumed);

/* Timed native-stack primitives. Word operations follow the 65C816 byte
 * order and optionally perform LakeSnes's checkInt between the two bytes. */
void     recomp_stack_push8(uint8_t value);
uint8_t  recomp_stack_pull8(void);
void     recomp_stack_push16(uint16_t value, bool check_between);
uint16_t recomp_stack_pull16(bool check_between);

/* Data accesses whose checkInt point is immediately before an 8-bit access
 * or between the low/high bytes of a 16-bit access. */
uint8_t  bus_read8_checked(uint8_t bank, uint16_t addr);
void     bus_write8_checked(uint8_t bank, uint16_t addr, uint8_t val);
uint16_t bus_read16_checked(uint8_t bank, uint16_t addr);
void     bus_write16_checked(uint8_t bank, uint16_t addr, uint16_t val);
void     bus_write16_reversed_checked(uint8_t bank, uint16_t addr,
                                      uint16_t val);

/* Direct WRAM access (faster than bus routing for stack/DP operations) */
uint8_t  bus_wram_read8(uint32_t offset);
void     bus_wram_write8(uint32_t offset, uint8_t val);
uint16_t bus_wram_read16(uint32_t offset);
void     bus_wram_write16(uint32_t offset, uint16_t val);

/* Get pointer to WRAM (128KB) for bulk access */
uint8_t *bus_get_wram(void);

/* Get pointer to ROM data */
const uint8_t *bus_get_rom(uint32_t *size_out);

/*
 * Direct SPC700 RAM access — for fast bulk uploads that bypass the
 * polled transfer protocol. Use this instead of the byte-by-byte
 * echo handshake when uploading large blocks to the audio CPU.
 */
void     bus_apu_write_ram(uint16_t spc_addr, uint8_t val);
uint8_t  bus_apu_read_ram(uint16_t spc_addr);

/*
 * Advance SNES master cycles — runs the SPC700 (and timers) forward.
 *
 * Recompiled code bypasses the normal CPU cycle counting, so polling loops
 * that wait on APU port responses (e.g., the SPC700 IPL upload protocol)
 * will deadlock unless you explicitly advance time.
 *
 * Call this inside tight polling loops that read APU ports ($2140-$2143).
 * A reasonable value is 8-64 master cycles per iteration (one 65816
 * instruction is typically 6-8 master cycles).
 */
void bus_run_cycles(int master_cycles);

/*
 * Super FX / GSU access — for recompiled code that needs to interact
 * with the GSU coprocessor directly (e.g., writing R15 to start execution,
 * reading status flags, setting up screen parameters).
 *
 * These are thin wrappers around gsu_read/gsu_write.
 * Returns 0 / no-op if no GSU is present.
 */
uint8_t bus_gsu_read(uint16_t addr);
void    bus_gsu_write(uint16_t addr, uint8_t val);

/* Trigger a GSU execution run (call after writing R15 high byte) */
void    bus_gsu_run(void);

/* Check if cartridge has a GSU */
bool    bus_has_gsu(void);

#endif /* SNESRECOMP_BUS_H */
