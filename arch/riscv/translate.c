/*
 *  RISC-V main translation routines
 *
 *  Author: Sagar Karandikar, sagark@eecs.berkeley.edu
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */
#include "cpu.h"

#include "instmap.h"
#include "debug.h"
#include "arch_callbacks.h"
#include "cpu_registers.h"

/* global register indices */
static TCGv cpu_gpr[32], cpu_pc, cpu_opcode;
static TCGv_i64 cpu_fpr[32]; /* assume F and D extensions */
static TCGv cpu_vstart;

//#define DL_TRACE_HUMAN_READABLE
//#define DL_TRACE_FULL
//#define DL_TRACE_COMPACT
#define DL_TRACE_ARITH

/* constants */
// 산술 연산 분류 코드 정의
// helper에 인자로 전달한다
enum DLInstClass {
    DL_RISC_ARITH_IMM = 0x00, 
    DL_RISC_ARITH, 
    DL_RISC_FMADD = 0x10,   // FP Multiply-Add; MAC 연산과 기능적으로 동일
    DL_RISC_FMSUB,          // FP Multiply-Subtract
    DL_RISC_FNMADD,         // FP Negative Multiply-Add; MAC의 결과에 부호 반전
    DL_RISC_FNMSUB,         // FP Negative Multiply-Subtract
    DL_RISC_FP_ARITH,       //
    // 벡터 산술연산: 피연산자별 분류{vv, vi, vx}
    DL_RISC_V_IVV = 0x20,
    DL_RISC_V_IVX,
    DL_RISC_V_IVI,
    DL_RISC_V_MVV,
    DL_RISC_V_MVX,
    DL_RISC_V_FVV,
    DL_RISC_V_FVF,
    DL_RISC_MEM = 0x30,     // int/FP load/store
    DL_RISC_VL_US = 0x31,   // unit-stride (vle)
    DL_RISC_VL_VS = 0x32,   // vector-strided (vlse)
    DL_RISC_VL_UVI = 0x33,  // unordered vector-indexed (vlxei)
    DL_RISC_VL_OVI = 0x33,  // ordered vector-indexed (vlxei)
    DL_RISC_VS_US = 0x34,   // unit-stride (vse)
    DL_RISC_VS_VS = 0x35,   // vector-strided (vsse)
    DL_RISC_VS_UVI = 0x36,  // unordered vector-indexed (vsxei)
    DL_RISC_VS_OVI = 0x36,  // ordered vector-indexed (vsxei)
    DL_RISC_UNKNOWN = 0xff,
    DL_RISC_CUSTOM0 = 0xf0,
    DL_RISC_CUSTOM1 = 0xf1
};

// DataLab: installed custom instructions
enum DLCustomInstructionIndex {
    DL_CUSTOM3_SPRINGBOK_SIMPRINT = 0,
    DL_CUSTOM3_SPRINGBOK_XCOUNT,
    DL_CUSTOM3_SPRINGBOK_HOSTREQ,
    DL_CUSTOM3_SPRINGBOK_FINISH,
    DL_CUSTOM0_DRBEGIN,
    DL_CUSTOM0_DREND
};

#define DL_CUSTOM_INSTRUCTION_COUNT 6
const char *DL_CUSTOM_INSTRUCTION_MNEMONIC[DL_CUSTOM_INSTRUCTION_COUNT] = {
    "simprint",
    "xcount",
    "hostreq",
    "finish",
    "dr.begin",
    "dr.end"
};

enum DLRiscvOpcodes {
    DL_OPCODE_CUSTOM0 = 0b0001011,
    DL_OPCODE_CUSTOM1 = 0b0101011,
    DL_OPCODE_CUSTOM2 = 0b1011011,
    DL_OPCODE_CUSTOM3 = 0b1111011
};


/* global register indices */
static TCGv cpu_gpr[32], cpu_pc, cpu_opcode;
static TCGv_i64 cpu_fpr[32]; /* assume F and D extensions */
static TCGv cpu_vstart;

/* DataLab: Datastructures for instruction trace */
// TODO: 64비트 정수로 확장?
typedef struct DLInstStat {
    uint32_t ldCnt;
    uint32_t lwCnt;
    uint32_t lwuCnt;
    uint32_t lhCnt;
    uint32_t lhuCnt;
    uint32_t lbCnt;
    uint32_t lbuCnt;

    uint32_t sdCnt;
    uint32_t swCnt;
    uint32_t shCnt;
    uint32_t sbCnt;

    uint32_t fldCnt;	
    uint32_t flwCnt;
    uint32_t flhCnt;
    uint32_t fsdCnt;
    uint32_t fswCnt;
    uint32_t fshCnt;

    // 8, 16, 32, 64
    uint32_t vleCnt[4];
    uint32_t vlseCnt[4];
    uint32_t vlxeiCnt[4];

    uint32_t vseCnt[4];
    uint32_t vsseCnt[4];
    uint32_t vsxeiCnt[4];

    uint32_t arithImmCnt;
    uint32_t arithCnt;

    // FP arith
    // operand precise: sigle, double, half
    uint32_t fmaddCnt[3];
    uint32_t fmsubCnt[3];
    uint32_t fnmaddCnt[3];
    uint32_t fnmsubCnt[3];
    uint32_t fparithCnt;

    // Vector arith
    uint32_t varithiCnt[3]; // vv, vx, vi
    uint32_t varithmCnt[2]; // vv, vx
    uint32_t varithfCnt[2]; // vv, vf

    uint32_t customCnt[DL_CUSTOM_INSTRUCTION_COUNT];
    uint32_t unknownCnt;
    uint64_t instCtr;
} DLInstStat;

/* Datastructure for binary format trace */
// opType과 dataType이 모두 비트 1로 지정되어 있으면 unknown이나 custom이다
// 이때 명령어 종류는 operandSize의 4비트 필드를 이용하여 식별한다 (DLTraceCustomCode 참조)
/* traceV2 */
typedef struct DLTraceLow {
	uint8_t opType: 2;			// load/store/arith/unknown (or custom)
	uint8_t dataType: 3;		// sint/uint/float/vector
	uint8_t operandSize: 3;		// 8/16/32/64/128
} DLTraceLow;

typedef struct DLTraceCompactMem {
	DLTraceLow lower;
	uint64_t instCtr;			// instruction counter
	uint32_t addr;				// target address (mem) / PC (arith)
} DLTraceCompactMem;

typedef struct DLTraceCompactArith {
    DLTraceLow lower;
    uint64_t instCtr;
    uint32_t addr;
    uint8_t opclass;            // 명령어 세부 분류
} DLTraceCompactArith;

#define DL_TRACE_SIZE_COMPACT_MEM   sizeof(struct DLTraceLow) + 12
#define DL_TRACE_SIZE_COMPACT_ARITH sizeof(struct DLTraceLow) + 13

/* DataLab: Global variables for memory trace */
DLInstStat instStat = { 0, };
const char *LOG_PATH_BASE="/home/euntae/tmp/renode-log/ecg_small";
//const char *LOG_PATH_BASE="/work/renode-logs/mobilebert";
FILE *logfp = NULL;
char pathName[256];

#include "tb-helper.h"

void translate_init(void)
{
    int i;

    static const char *const regnames[] = {
        "zero", "ra  ", "sp  ", "gp  ", "tp  ", "t0  ",  "t1  ",  "t2  ", "s0  ", "s1  ", "a0  ", "a1  ", "a2  ", "a3  ",  "a4  ",
        "a5  ", "a6  ", "a7  ", "s2  ", "s3  ", "s4  ", "s5  ",  "s6  ",  "s7  ", "s8  ", "s9  ", "s10 ", "s11 ", "t3  ", "t4  ",
        "t5  ",  "t6  "
    };

    static const char *const fpr_regnames[] = {
        "ft0", "ft1", "ft2",  "ft3",  "ft4", "ft5", "ft6",  "ft7", "fs0", "fs1", "fa0",  "fa1",  "fa2", "fa3", "fa4",  "fa5",
        "fa6", "fa7", "fs2",  "fs3",  "fs4", "fs5", "fs6",  "fs7", "fs8", "fs9", "fs10", "fs11", "ft8", "ft9", "ft10", "ft11"
    };

    /* cpu_gpr[0] is a placeholder for the zero register. Do not use it. */
    /* Use the gen_set_gpr and gen_get_gpr helper functions when accessing */
    /* registers, unless you specifically block reads/writes to reg 0 */
    TCGV_UNUSED(cpu_gpr[0]);
    for (i = 1; i < 32; i++) {
        cpu_gpr[i] = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, gpr[i]), regnames[i]);
    }

    for (i = 0; i < 32; i++) {
        cpu_fpr[i] = tcg_global_mem_new_i64(TCG_AREG0, offsetof(CPUState, fpr[i]), fpr_regnames[i]);
    }

    cpu_pc = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, pc), "pc");
    cpu_opcode = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, opcode), "opcode");
    cpu_vstart = tcg_global_mem_new(TCG_AREG0, offsetof(CPUState, vstart), "vstart");

    /* DataLab: execution trace initialize */
    time_t timer;
    struct tm *ts;
    timer = time(NULL);
    ts = localtime(&timer);
    
#ifdef DL_TRACE_HUMAN_READABLE
    sprintf(pathName, "%s_%d%02d%02d_%02d%02d%02d.txt", LOG_PATH_BASE, 
        ts->tm_year + 1900, ts->tm_mon + 1, ts->tm_mday,
        ts->tm_hour, ts->tm_min, ts->tm_sec);
    logfp = fopen(pathName, "wt");
#else
    sprintf(pathName, "%s_%d%02d%02d_%02d%02d%02d.bin", LOG_PATH_BASE, 
        ts->tm_year + 1900, ts->tm_mon + 1, ts->tm_mday,
        ts->tm_hour, ts->tm_min, ts->tm_sec);
    logfp = fopen(pathName, "wb");
    printf("\ntranslate_init(): sizeof(DLTraceLow)=%zu\n", sizeof(DLTraceLow));
#endif
    printf("\ntranslate_init(): %s is created\n", pathName);
}

static inline void kill_unknown(DisasContext *dc, int excp);

// Other values are defined in `exec-all.h`
#define DISAS_STOP 4    /* Need to exit tb for syscall, sret, etc. */
#define DISAS_NONE 5    /* When seen outside of translation while loop, indicates need to exit tb due to end of page. */
#define DISAS_BRANCH 6  /* Need to exit tb for branch, jal, etc. */

#ifdef TARGET_RISCV64
#define CASE_OP_32_64(X) case X: case glue(X, W)
#define BITMANIP_SHAMT_MASK 0x3F
#else
#define CASE_OP_32_64(X) case X
#define BITMANIP_SHAMT_MASK 0x1F
#endif

// RISC-V User ISA, Release 2.2, section 1.2 Instruction Length Encoding
static int decode_instruction_length(uint64_t opcode)
{
    int instruction_length = 0;
    if ((opcode & 0b11) != 0b11) {
        instruction_length = 2;
    } else if ((opcode & 0b11100) != 0b11100) {
        instruction_length = 4;
    } else if ((opcode & 0b111111) == 0b011111) {
        instruction_length = 6;
    } else if ((opcode & 0b1111111) == 0b0111111) {
        instruction_length = 8;
    } else if (extract32(opcode, 12, 3) != 0b111) {
        instruction_length = 10 + 2 * extract32(opcode, 12, 3);
    } else {
        // Reserved for >=192 bits, this function returns 0 in that case.
    }
    return instruction_length;
}

static inline uint64_t format_opcode(uint64_t opcode, int instruction_length)
{
    return opcode & (((typeof(opcode))1 << (8 * instruction_length)) - 1);
}

static int ensure_extension(DisasContext *dc, target_ulong ext)
{
    if (riscv_has_ext(cpu, ext)) {
        return 1;
    }

    if (!riscv_silent_ext(cpu, ext)) {
        char letter = 0;
        riscv_features_to_string(ext, &letter, 1);

        int instruction_length = decode_instruction_length(dc->opcode);
        tlib_printf(LOG_LEVEL_ERROR, "RISC-V '%c' instruction set is not enabled for this CPU! PC: 0x%llx, opcode: 0x%0*llx",
                    letter, dc->base.pc, /* padding */ 2 * instruction_length, format_opcode(dc->opcode, instruction_length));
    }

    kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
    return 0;
}

static int ensure_additional_extension(DisasContext *dc, target_ulong ext)
{
    if (riscv_has_additional_ext(cpu, ext)) {
        return 1;
    }

    char *encoding = NULL;
    switch(ext) {
    case RISCV_FEATURE_ZBA:
        encoding = "ba";
        break;
    case RISCV_FEATURE_ZBB:
        encoding = "bb";
        break;
    case RISCV_FEATURE_ZBC:
        encoding = "bc";
        break;
    case RISCV_FEATURE_ZBS:
        encoding = "bs";
        break;
    case RISCV_FEATURE_ZICSR:
        encoding = "icsr";
        break;
    case RISCV_FEATURE_ZIFENCEI:
        encoding = "ifencei";
        break;
    case RISCV_FEATURE_ZFH:
        encoding = "fh";
        break;
    default:
        tlib_printf(LOG_LEVEL_ERROR, "Unexpected additional extension encoding: %d", ext);
        break;
    }

    int instruction_length = decode_instruction_length(dc->opcode);
    tlib_printf(LOG_LEVEL_ERROR, "RISC-V Z%s instruction set is not enabled for this CPU! PC: 0x%llx, opcode: 0x%0*llx",
            encoding, dc->base.pc, /* padding */ 2 * instruction_length, format_opcode(dc->opcode, instruction_length));
    
    kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
    return 0;
}

static int ensure_fp_extension(DisasContext *dc, int precision_bit)
{
    switch((enum riscv_floating_point_precision)extract64(dc->opcode, precision_bit, 2))
    {
        case RISCV_HALF_PRECISION:
            return ensure_additional_extension(dc, RISCV_FEATURE_ZFH);
        case RISCV_SINGLE_PRECISION:
            return ensure_extension(dc, RISCV_FEATURE_RVF);
        case RISCV_DOUBLE_PRECISION:
            return ensure_extension(dc, RISCV_FEATURE_RVD);
        default:
            tlib_printf(LOG_LEVEL_ERROR, "Unknown floating point instruction encoding! PC: 0x%llx", dc->base.pc);
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            return false;
    }
}

static int ensure_fp_extension_for_load_store(DisasContext *dc)
{
    switch(extract64(dc->opcode, 12, 3))
    {
        case 1:
            return ensure_additional_extension(dc, RISCV_FEATURE_ZFH);
        case 2:
            return ensure_extension(dc, RISCV_FEATURE_RVF);
        case 3:
            return ensure_extension(dc, RISCV_FEATURE_RVD);
        default:
            tlib_printf(LOG_LEVEL_ERROR, "Unknown floating point instruction encoding! PC: 0x%llx", dc->base.pc);
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            return false;
    }
}

static inline void gen_sync_pc(DisasContext *dc)
{
    tcg_gen_movi_tl(cpu_pc, dc->base.pc);
    tcg_gen_movi_tl(cpu_opcode, dc->opcode);
}

static inline void generate_exception(DisasContext *dc, int excp)
{
    gen_sync_pc(dc);
    TCGv_i32 helper_tmp = tcg_const_i32(excp);
    gen_helper_raise_exception(cpu_env, helper_tmp);
    tcg_temp_free_i32(helper_tmp);
}

static inline void generate_exception_mbadaddr(DisasContext *dc, int excp)
{
    gen_sync_pc(dc);
    TCGv_i32 helper_tmp = tcg_const_i32(excp);
    gen_helper_raise_exception_mbadaddr(cpu_env, helper_tmp, cpu_pc);
    tcg_temp_free_i32(helper_tmp);
}

/* unknown instruction */
static inline void kill_unknown(DisasContext *dc, int excp)
{
    gen_sync_pc(dc);

    // According to the RISC-V ISA manual,
    // for Illegal Instruction, mtval
    // should contain an opcode of the faulting instruction.
    TCGv_i32 helper_tmp = tcg_const_i32(excp);
    TCGv_i32 helper_bdinstr = tcg_const_i32(dc->opcode);
    gen_helper_raise_exception_mbadaddr(cpu_env, helper_tmp, helper_bdinstr);
    tcg_temp_free_i32(helper_tmp);
    tcg_temp_free_i32(helper_bdinstr);

    dc->base.is_jmp = DISAS_STOP;
}

static inline bool use_goto_tb(DisasContext *dc, target_ulong dest)
{
    return (dc->base.tb->pc & TARGET_PAGE_MASK) == (dest & TARGET_PAGE_MASK);
}

static inline void gen_goto_tb(DisasContext *dc, int n, target_ulong dest)
{
    if (use_goto_tb(dc, dest)) {
        /* chaining is only allowed when the jump is to the same page */
        tcg_gen_goto_tb(n);
        tcg_gen_movi_tl(cpu_pc, dest);
        gen_exit_tb(dc->base.tb, n);
    } else {
        tcg_gen_movi_tl(cpu_pc, dest);
        gen_exit_tb_no_chaining(dc->base.tb);
    }
}

static inline void try_run_gpr_access_hook(int reg_num, int is_write)
{
    if(unlikely(env->are_post_gpr_access_hooks_enabled))
    {
        if(env->post_gpr_access_hook_mask & (1u << reg_num))
        {
            TCGv_i32 register_index = tcg_const_i32(reg_num);
            TCGv_i32 is_write_const = tcg_const_i32(is_write);
            gen_helper_handle_post_gpr_access_hook(register_index, is_write_const);
            tcg_temp_free_i32(register_index);
            tcg_temp_free_i32(is_write_const);
        }
    }
}

/* Wrapper for getting reg values - need to check of reg is zero since
 * cpu_gpr[0] is not actually allocated
 */
static inline void gen_get_gpr(TCGv t, int reg_num)
{
    try_run_gpr_access_hook(reg_num, 0);

    if (reg_num == 0) {
        tcg_gen_movi_tl(t, 0);
    } else {
        tcg_gen_mov_tl(t, cpu_gpr[reg_num]);
    }
}

#if defined(TARGET_RISCV32)
static inline uint32_t get_gpr_raw(int reg_num)
#elif defined(TARGET_RISCV64)
static inline uint64_t get_gpr_raw(int reg_num)
#endif
{
    return cpu->gpr[reg_num];
}

static inline void gen_get_fpr(TCGv_i64 t, int reg_num)
{
    tcg_gen_mov_tl(t, cpu_fpr[reg_num]);
}

/* Wrapper for setting reg values - need to check of reg is zero since
 * cpu_gpr[0] is not actually allocated. this is more for safety purposes,
 * since we usually avoid calling the OP_TYPE_gen function if we see a write to
 * $zero
 */
static inline void gen_set_gpr(int reg_num_dst, TCGv t)
{
    if (reg_num_dst != 0) {
        tcg_gen_mov_tl(cpu_gpr[reg_num_dst], t);
    }

    try_run_gpr_access_hook(reg_num_dst, 1);
}

static inline void get_set_gpr_imm(int reg_num_dst, target_ulong value)
{
    if (reg_num_dst != 0) {
        tcg_gen_movi_tl(cpu_gpr[reg_num_dst], value);
    }

    try_run_gpr_access_hook(reg_num_dst, 1);
}

/* Some instructions don't allow NFIELDS value to be different from 1, 2, 4 or 8.
 * As NFIELDS can be expressed as `nf + 1` this function checks if the above condition is true, while saving a few clock cycles
 * */
static inline bool is_nfields_power_of_two(uint32_t nf)
{
    return (nf & (nf + 1)) == 0;
}

static inline void generate_vill_check(DisasContext *dc)
{
    TCGv t0 = tcg_temp_local_new();
    int done = gen_new_label();

    tcg_gen_ld_tl(t0, cpu_env, offsetof(CPUState, vill));
    tcg_gen_brcondi_tl(TCG_COND_EQ, t0, 0x0, done);

    kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);

    gen_set_label(done);
    tcg_temp_free(t0);
}

static void gen_mulhsu(TCGv ret, TCGv arg1, TCGv arg2)
{
    TCGv rl = tcg_temp_new();
    TCGv rh = tcg_temp_new();

    tcg_gen_mulu2_tl(rl, rh, arg1, arg2);
    /* fix up for one negative */
    tcg_gen_sari_tl(rl, arg1, TARGET_LONG_BITS - 1);
    tcg_gen_and_tl(rl, rl, arg2);
    tcg_gen_sub_tl(ret, rh, rl);

    tcg_temp_free(rl);
    tcg_temp_free(rh);
}

static void gen_fsgnj(DisasContext *dc, uint32_t rd, uint32_t rs1, uint32_t rs2, int rm, enum riscv_floating_point_precision precision)
{
    TCGv t0 = tcg_temp_new();
    int fp_ok = gen_new_label();
    int done = gen_new_label();

    int64_t sign_mask = get_float_sign_mask(precision);

    // check MSTATUS.FS
    tcg_gen_ld_tl(t0, cpu_env, offsetof(CPUState, mstatus));
    tcg_gen_andi_tl(t0, t0, MSTATUS_FS);
    tcg_gen_brcondi_tl(TCG_COND_NE, t0, 0x0, fp_ok);
    // MSTATUS_FS field was zero:
    kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
    tcg_gen_br(done);

    // proceed with operation
    gen_set_label(fp_ok);
    TCGv_i64 src1 = tcg_temp_local_new_i64();
    TCGv_i64 src2 = tcg_temp_new_i64();

    gen_unbox_float(precision, env, src1, cpu_fpr[rs1]);
    tcg_gen_mov_i64(src2, cpu_fpr[rs2]);

    switch (rm) {
    case 0:               /* fsgnj */

        if (rs1 == rs2) { /* FMOV */
            tcg_gen_mov_i64(cpu_fpr[rd], src1);
        }

        tcg_gen_andi_i64(src1, src1, ~sign_mask);
        tcg_gen_andi_i64(src2, src2, sign_mask);
        tcg_gen_or_i64(cpu_fpr[rd], src1, src2);
        gen_box_float(precision, cpu_fpr[rd]);
        break;
    case 1: /* fsgnjn */
        tcg_gen_andi_i64(src1, src1, ~sign_mask);
        tcg_gen_not_i64(src2, src2);
        tcg_gen_andi_i64(src2, src2, sign_mask);
        tcg_gen_or_i64(cpu_fpr[rd], src1, src2);
        gen_box_float(precision, cpu_fpr[rd]);
        break;
    case 2: /* fsgnjx */
        tcg_gen_andi_i64(src2, src2, sign_mask);
        tcg_gen_xor_i64(cpu_fpr[rd], src1, src2);
        gen_box_float(precision, cpu_fpr[rd]);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
    }

    tcg_temp_free_i64(src1);
    tcg_temp_free_i64(src2);
    gen_set_label(done);
    tcg_temp_free(t0);
}

static void gen_arith(DisasContext *dc, uint32_t opc, int rd, int rs1, int rs2)
{
    TCGv source1, source2, cond1, cond2, zeroreg, resultopt1;
    source1 = tcg_temp_new();
    source2 = tcg_temp_new();
    gen_get_gpr(source1, rs1);
    gen_get_gpr(source2, rs2);

    switch (opc) {
        CASE_OP_32_64(OPC_RISC_ADD) :
            tcg_gen_add_tl(source1, source1, source2);
        break;
        CASE_OP_32_64(OPC_RISC_SUB) :
            tcg_gen_sub_tl(source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SLLW:
        tcg_gen_andi_tl(source2, source2, 0x1F);
        tcg_gen_shl_tl(source1, source1, source2);
        break;
#endif
    case OPC_RISC_SLL:
        tcg_gen_andi_tl(source2, source2, TARGET_LONG_BITS - 1);
        tcg_gen_shl_tl(source1, source1, source2);
        break;
    case OPC_RISC_SLT:
        tcg_gen_setcond_tl(TCG_COND_LT, source1, source1, source2);
        break;
    case OPC_RISC_SLTU:
        tcg_gen_setcond_tl(TCG_COND_LTU, source1, source1, source2);
        break;
    case OPC_RISC_XOR:
        tcg_gen_xor_tl(source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SRLW:
        /* clear upper 32 */
        tcg_gen_ext32u_tl(source1, source1);
        tcg_gen_andi_tl(source2, source2, 0x1F);
        tcg_gen_shr_tl(source1, source1, source2);
        break;
#endif
    case OPC_RISC_SRL:
        tcg_gen_andi_tl(source2, source2, TARGET_LONG_BITS - 1);
        tcg_gen_shr_tl(source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SRAW:
        /* first, trick to get it to act like working on 32 bits (get rid of
           upper 32, sign extend to fill space) */
        tcg_gen_ext32s_tl(source1, source1);
        tcg_gen_andi_tl(source2, source2, 0x1F);
        tcg_gen_sar_tl(source1, source1, source2);
        break;
        /* fall through to SRA */
#endif
    case OPC_RISC_SRA:
        tcg_gen_andi_tl(source2, source2, TARGET_LONG_BITS - 1);
        tcg_gen_sar_tl(source1, source1, source2);
        break;
    case OPC_RISC_OR:
        tcg_gen_or_tl(source1, source1, source2);
        break;
    case OPC_RISC_AND:
        tcg_gen_and_tl(source1, source1, source2);
        break;
        CASE_OP_32_64(OPC_RISC_MUL) :
            tcg_gen_mul_tl(source1, source1, source2);
        break;
    case OPC_RISC_MULH:
        tcg_gen_muls2_tl(source2, source1, source1, source2);
        break;
    case OPC_RISC_MULHSU:
        gen_mulhsu(source1, source1, source2);
        break;
    case OPC_RISC_MULHU:
        tcg_gen_mulu2_tl(source2, source1, source1, source2);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_DIVW:
        tcg_gen_ext32s_tl(source1, source1);
        tcg_gen_ext32s_tl(source2, source2);
        /* fall through to DIV */
#endif
    /* fallthrough */
    case OPC_RISC_DIV:
        /* Handle by altering args to tcg_gen_div to produce req'd results:
         * For overflow: want source1 in source1 and 1 in source2
         * For div by zero: want -1 in source1 and 1 in source2 -> -1 result */
        cond1 = tcg_temp_new();
        cond2 = tcg_temp_new();
        zeroreg = tcg_const_tl(0);
        resultopt1 = tcg_temp_new();

        tcg_gen_movi_tl(resultopt1, (target_ulong) - 1);
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond2, source2, (target_ulong)(~0L));
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond1, source1, ((target_ulong)1) << (TARGET_LONG_BITS - 1));
        tcg_gen_and_tl(cond1, cond1, cond2);                 /* cond1 = overflow */
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond2, source2, 0); /* cond2 = div 0 */
        /* if div by zero, set source1 to -1, otherwise don't change */
        tcg_gen_movcond_tl(TCG_COND_EQ, source1, cond2, zeroreg, source1, resultopt1);
        /* if overflow or div by zero, set source2 to 1, else don't change */
        tcg_gen_or_tl(cond1, cond1, cond2);
        tcg_gen_movi_tl(resultopt1, (target_ulong)1);
        tcg_gen_movcond_tl(TCG_COND_EQ, source2, cond1, zeroreg, source2, resultopt1);
        tcg_gen_div_tl(source1, source1, source2);

        tcg_temp_free(cond1);
        tcg_temp_free(cond2);
        tcg_temp_free(zeroreg);
        tcg_temp_free(resultopt1);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_DIVUW:
        tcg_gen_ext32u_tl(source1, source1);
        tcg_gen_ext32u_tl(source2, source2);
        /* fall through to DIVU */
#endif
    /* fallthrough */
    case OPC_RISC_DIVU:
        cond1 = tcg_temp_new();
        zeroreg = tcg_const_tl(0);
        resultopt1 = tcg_temp_new();

        tcg_gen_setcondi_tl(TCG_COND_EQ, cond1, source2, 0);
        tcg_gen_movi_tl(resultopt1, (target_ulong) - 1);
        tcg_gen_movcond_tl(TCG_COND_EQ, source1, cond1, zeroreg, source1, resultopt1);
        tcg_gen_movi_tl(resultopt1, (target_ulong)1);
        tcg_gen_movcond_tl(TCG_COND_EQ, source2, cond1, zeroreg, source2, resultopt1);
        tcg_gen_divu_tl(source1, source1, source2);

        tcg_temp_free(cond1);
        tcg_temp_free(zeroreg);
        tcg_temp_free(resultopt1);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_REMW:
        tcg_gen_ext32s_tl(source1, source1);
        tcg_gen_ext32s_tl(source2, source2);
        /* fall through to REM */
#endif
    /* fallthrough */
    case OPC_RISC_REM:
        cond1 = tcg_temp_new();
        cond2 = tcg_temp_new();
        zeroreg = tcg_const_tl(0);
        resultopt1 = tcg_temp_new();

        tcg_gen_movi_tl(resultopt1, 1L);
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond2, source2, (target_ulong) - 1);
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond1, source1, (target_ulong)1 << (TARGET_LONG_BITS - 1));
        tcg_gen_and_tl(cond2, cond1, cond2);                 /* cond1 = overflow */
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond1, source2, 0); /* cond2 = div 0 */
        /* if overflow or div by zero, set source2 to 1, else don't change */
        tcg_gen_or_tl(cond2, cond1, cond2);
        tcg_gen_movcond_tl(TCG_COND_EQ, source2, cond2, zeroreg, source2, resultopt1);
        tcg_gen_rem_tl(resultopt1, source1, source2);
        /* if div by zero, just return the original dividend */
        tcg_gen_movcond_tl(TCG_COND_EQ, source1, cond1, zeroreg, resultopt1, source1);

        tcg_temp_free(cond1);
        tcg_temp_free(cond2);
        tcg_temp_free(zeroreg);
        tcg_temp_free(resultopt1);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_REMUW:
        tcg_gen_ext32u_tl(source1, source1);
        tcg_gen_ext32u_tl(source2, source2);
        /* fall through to REMU */
#endif
    /* fallthrough */
    case OPC_RISC_REMU:
        cond1 = tcg_temp_new();
        zeroreg = tcg_const_tl(0);
        resultopt1 = tcg_temp_new();

        tcg_gen_movi_tl(resultopt1, (target_ulong)1);
        tcg_gen_setcondi_tl(TCG_COND_EQ, cond1, source2, 0);
        tcg_gen_movcond_tl(TCG_COND_EQ, source2, cond1, zeroreg, source2, resultopt1);
        tcg_gen_remu_tl(resultopt1, source1, source2);
        /* if div by zero, just return the original dividend */
        tcg_gen_movcond_tl(TCG_COND_EQ, source1, cond1, zeroreg, resultopt1, source1);

        tcg_temp_free(cond1);
        tcg_temp_free(zeroreg);
        tcg_temp_free(resultopt1);
        break;
    case OPC_RISC_ADD_UW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBA)) {
            return;
        }
        tcg_gen_andi_tl(source1, source1, 0xFFFFFFFF);
        tcg_gen_add_tl(source1, source1, source2);
        break;
    case OPC_RISC_SH1ADD:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBA)) {
            return;
        }
        tcg_gen_shli_tl(source1, source1, 1);
        tcg_gen_add_tl(source1, source1, source2);
        break;
    case OPC_RISC_SH1ADD_UW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBA)) {
            return;
        }
        tcg_gen_andi_tl(source1, source1, 0xFFFFFFFF);
        tcg_gen_shli_tl(source1, source1, 1);
        tcg_gen_add_tl(source1, source1, source2);
        break;
    case OPC_RISC_SH2ADD:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBA)) {
            return;
        }
        tcg_gen_shli_tl(source1, source1, 2);
        tcg_gen_add_tl(source1, source1, source2);
        break;
    case OPC_RISC_SH2ADD_UW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBA)) {
            return;
        }
        tcg_gen_andi_tl(source1, source1, 0xFFFFFFFF);
        tcg_gen_shli_tl(source1, source1, 2);
        tcg_gen_add_tl(source1, source1, source2);
        break;
    case OPC_RISC_SH3ADD:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBA)) {
            return;
        }
        tcg_gen_shli_tl(source1, source1, 3);
        tcg_gen_add_tl(source1, source1, source2);
        break;
    case OPC_RISC_SH3ADD_UW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBA)) {
            return;
        }
        tcg_gen_andi_tl(source1, source1, 0xFFFFFFFF);
        tcg_gen_shli_tl(source1, source1, 3);
        tcg_gen_add_tl(source1, source1, source2);
        break;
     case OPC_RISC_ANDN:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_not_tl(source2, source2);
        tcg_gen_and_tl(source1, source1, source2);
        break;
    case OPC_RISC_ORN:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_not_tl(source2, source2);
        tcg_gen_or_tl(source1, source1, source2);
        break;
    case OPC_RISC_XNOR:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_xor_tl(source1, source1, source2);
        tcg_gen_not_tl(source1, source1);
        break;
    case OPC_RISC_MAX:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
#if defined(TARGET_RISCV32)
        tcg_gen_smax_i32(source1, source1, source2);
#elif defined(TARGET_RISCV64)
        tcg_gen_smax_i64(source1, source1, source2);
#endif
        break;
    case OPC_RISC_MAXU:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
#if defined(TARGET_RISCV32)
        tcg_gen_umax_i32(source1, source1, source2);
#elif defined(TARGET_RISCV64)
        tcg_gen_umax_i64(source1, source1, source2);
#endif
        break;
    case OPC_RISC_MIN:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
#if defined(TARGET_RISCV32)
        tcg_gen_smin_i32(source1, source1, source2);
#elif defined(TARGET_RISCV64)
        tcg_gen_smin_i64(source1, source1, source2);
#endif
        break;
    case OPC_RISC_MINU:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
#if defined(TARGET_RISCV32)
        tcg_gen_umin_i32(source1, source1, source2);
#elif defined(TARGET_RISCV64)
        tcg_gen_umin_i64(source1, source1, source2);
#endif
        break;
    case OPC_RISC_ZEXT_H_32:
    case OPC_RISC_ZEXT_H_64:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_andi_tl(source1, source1, 0xFFFF);
        break;
    case OPC_RISC_ROL:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
#if defined(TARGET_RISCV32)
        TCGv left = tcg_temp_new_internal_i32(0);
        TCGv right = tcg_temp_new_internal_i32(0);
        TCGv xlen = tcg_temp_new_internal_i32(0);
        tcg_gen_movi_tl(xlen, 32);
        tcg_gen_andi_tl(source2, source2, BITMANIP_SHAMT_MASK);
#elif defined(TARGET_RISCV64)
        TCGv left = tcg_temp_new_internal_i64(0);
        TCGv right = tcg_temp_new_internal_i64(0);
        TCGv xlen = tcg_temp_new_internal_i64(0);
        tcg_gen_movi_tl(xlen, 64);
        tcg_gen_andi_tl(source2, source2, BITMANIP_SHAMT_MASK);
#endif
        tcg_gen_movi_tl(left, 0UL);
        tcg_gen_movi_tl(right, 0UL);
        
        tcg_gen_shl_tl(left, source1, source2);
        tcg_gen_sub_tl(source2, xlen, source2);
        tcg_gen_shr_tl(right, source1, source1);
        tcg_gen_or_tl(source1, left, right);
        
        tcg_temp_free(left);
        tcg_temp_free(right);
        tcg_temp_free(xlen);
        break;
    case OPC_RISC_ROLW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, sextract64(rol32(get_gpr_raw(rs1), get_gpr_raw(rs2) & BITMANIP_SHAMT_MASK), 0, 32));
        break;
    case OPC_RISC_ROR:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
#if defined(TARGET_RISCV32)
        tcg_gen_movi_tl(source1, ror32(get_gpr_raw(rs1), get_gpr_raw(rs2) & BITMANIP_SHAMT_MASK));
#elif defined(TARGET_RISCV64)
        tcg_gen_movi_tl(source1, ror64(get_gpr_raw(rs1), get_gpr_raw(rs2) & BITMANIP_SHAMT_MASK));
#endif
        break;
    case OPC_RISC_RORW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, sextract64(ror32(get_gpr_raw(rs1), get_gpr_raw(rs2) & BITMANIP_SHAMT_MASK), 0, 32));
        break;
    case OPC_RISC_BCLR:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
#if defined(TARGET_RISCV32)
        TCGv bclr_t = tcg_temp_new_internal_i32(0);
        tcg_gen_andi_tl(source2, source2, BITMANIP_SHAMT_MASK);
#elif defined(TARGET_RISCV64)
        TCGv bclr_t = tcg_temp_new_internal_i64(0);
        tcg_gen_andi_tl(source2, source2, BITMANIP_SHAMT_MASK);
#endif
        tcg_gen_movi_tl(bclr_t, 1);
        tcg_gen_shl_tl(bclr_t, bclr_t, source2);
#if defined(TARGET_RISCV32)
        tcg_gen_not_i32(bclr_t, bclr_t);
#elif defined(TARGET_RISCV64)
        tcg_gen_not_i64(bclr_t, bclr_t);
#endif
        tcg_gen_and_tl(source1, source1, bclr_t);
        tcg_temp_free(bclr_t);
        break;
    case OPC_RISC_BEXT:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
#if defined(TARGET_RISCV32)
        tcg_gen_andi_tl(source2, source2, BITMANIP_SHAMT_MASK);
#elif defined(TARGET_RISCV64)
        tcg_gen_andi_tl(source2, source2, BITMANIP_SHAMT_MASK);
#endif
        tcg_gen_shr_tl(source1, source1, source2);
        tcg_gen_andi_tl(source1, source1, 1);
        break;
    case OPC_RISC_BINV:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
#if defined(TARGET_RISCV32)
        TCGv binv_t = tcg_temp_new_internal_i32(0);
        tcg_gen_andi_tl(source2, source2, BITMANIP_SHAMT_MASK);
#elif defined(TARGET_RISCV64)
        TCGv binv_t = tcg_temp_new_internal_i64(0);
        tcg_gen_andi_tl(source2, source2, BITMANIP_SHAMT_MASK);
#endif
        tcg_gen_movi_tl(binv_t, 1);
        tcg_gen_shl_tl(binv_t, binv_t, source2);
        tcg_gen_xor_tl(source1, source1, binv_t);

        tcg_temp_free(binv_t);
        break;
    case OPC_RISC_BSET:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
#if defined(TARGET_RISCV32)
        TCGv test = tcg_temp_new_internal_i32(0);
        tcg_gen_andi_tl(source2, source2, BITMANIP_SHAMT_MASK);
#elif defined(TARGET_RISCV64)
        TCGv test = tcg_temp_new_internal_i64(0);
        tcg_gen_andi_tl(source2, source2, BITMANIP_SHAMT_MASK);
#endif
        tcg_gen_movi_tl(test, 1UL);
        tcg_gen_shl_tl(test, test, source2);
        tcg_gen_or_tl(source1, source1, test);

        tcg_temp_free(test);
        break;
    case OPC_RISC_CLMUL:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBC)) {
            return;
        }
#if defined(TARGET_RISCV32)
        tcg_gen_movi_tl(source1, clmul32(get_gpr_raw(rs1), get_gpr_raw(rs2)));
#elif defined(TARGET_RISCV64)
        tcg_gen_movi_tl(source1, clmul64(get_gpr_raw(rs1), get_gpr_raw(rs2)));
#endif
        break;
    case OPC_RISC_CLMULR:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBC)) {
            return;
        }
#if defined(TARGET_RISCV32)
        tcg_gen_movi_tl(source1, clmulr32(get_gpr_raw(rs1), get_gpr_raw(rs2)));
#elif defined(TARGET_RISCV64)
        tcg_gen_movi_tl(source1, clmulr64(get_gpr_raw(rs1), get_gpr_raw(rs2)));
#endif
        break;
    case OPC_RISC_CLMULH:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBC)) {
            return;
        }
#if defined(TARGET_RISCV32)
        tcg_gen_movi_tl(source1, clmulh32(get_gpr_raw(rs1), get_gpr_raw(rs2)));
#elif defined(TARGET_RISCV64)
        tcg_gen_movi_tl(source1, clmulh64(get_gpr_raw(rs1), get_gpr_raw(rs2)));
#endif
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    if (opc & 0x8) { /* sign extend for W instructions */
        tcg_gen_ext32s_tl(source1, source1);
    }

#ifdef DL_TRACE_ARITH
    /* DataLab: helper call and free temporary variable */
    gen_sync_pc(dc);
    TCGv_i32 topc = tcg_const_i32(opc);
    TCGv_i32 topclass = tcg_const_i32(DL_RISC_ARITH);
    gen_helper_log_inst_arith(cpu_env, topc, topclass);
    tcg_temp_free(topc);
    tcg_temp_free(topclass);
    //==================================================//
#endif

    gen_set_gpr(rd, source1);
    tcg_temp_free(source1);
    tcg_temp_free(source2);
}

static void gen_synch(DisasContext *dc, uint32_t opc)
{
    switch (opc) {
    case OPC_RISC_FENCE:
        /* standard fence = NOP */
        break;
    case OPC_RISC_FENCE_I:
        if (!riscv_has_additional_ext(cpu, RISCV_FEATURE_ZIFENCEI)) {
            int instruction_length = decode_instruction_length(dc->opcode);
            tlib_printf(LOG_LEVEL_ERROR, "RISC-V Zifencei instruction set is not enabled for this CPU! In future release this configuration will lead to an illegal instruction exception. PC: 0x%llx, opcode: 0x%0*llx",
                dc->base.pc, /* padding */ 2 * instruction_length, format_opcode(dc->opcode, instruction_length));
        }
        gen_helper_fence_i(cpu_env);
        tcg_gen_movi_tl(cpu_pc, dc->npc);
        gen_exit_tb_no_chaining(dc->base.tb);
        dc->base.is_jmp = DISAS_BRANCH;
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
}

static void gen_arith_bitmanip(DisasContext *dc, int rd, int rs1, target_long imm, TCGv source1)
{
    uint32_t opc = 0;
    switch ((dc->opcode >> 12) & 0x7) {
    case 0x1:
        switch ((dc->opcode >> 26) & BITMANIP_SHAMT_MASK) {
        case 0b011000:
            opc = MASK_OP_ARITH_IMM_ZB_1_12(dc->opcode);
            break;
        case 0b010010:
        case 0b011010:
        case 0b001010:
            opc = MASK_OP_ARITH_IMM_ZB_1_12_SHAMT(dc->opcode);
            break;
        }
        break;
    case 0x5:
        switch ((dc->opcode >> 26) & BITMANIP_SHAMT_MASK) {
        case 0b001010:
        case 0b011010:
            opc = MASK_OP_ARITH_IMM_ZB_5_12(dc->opcode);
            break;
        case 0b010010:
        case 0b011000:
            opc = MASK_OP_ARITH_IMM_ZB_5_12_SHAMT(dc->opcode);
            break;
        }
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    switch (opc) {
    case OPC_RISC_CLZW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_clzi_i32(source1, source1, 32);
        break;
    case OPC_RISC_CTZW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, ctz32(get_gpr_raw(rs1)));
        break;
    case OPC_RISC_CPOPW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, ctpop32(get_gpr_raw(rs1)));
        break;
    case OPC_RISC_REV8_32:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, brev32(get_gpr_raw(rs1)));
        break;
    case OPC_RISC_REV8_64:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, brev64(get_gpr_raw(rs1)));
        break;
#if defined(TARGET_RISCV32)
    case OPC_RISC_CLZ:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_clzi_i32(source1, source1, 32);
        break;
    case OPC_RISC_CTZ:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, ctz32(get_gpr_raw(rs1)));
        break;
    case OPC_RISC_CPOP:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, ctpop32(get_gpr_raw(rs1)));
        break;
    case OPC_RISC_SEXT_B:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_sextract_i32(source1, source1, 0, 8);
        break;
    case OPC_RISC_SEXT_H:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_sextract_i32(source1, source1, 0, 16);
        break;
    case OPC_RISC_ORC_B:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, orcb32(get_gpr_raw(rs1)));
        break;
    case OPC_RISC_BCLRI:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
        tcg_gen_andi_tl(source1, source1, ~(1UL << (imm & BITMANIP_SHAMT_MASK)));
        break;
    case OPC_RISC_BEXTI:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
        tcg_gen_shri_tl(source1, source1, imm & BITMANIP_SHAMT_MASK);
        tcg_gen_andi_tl(source1, source1, 1UL);
        break;
    case OPC_RISC_BINVI:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
        tcg_gen_xori_tl(source1, source1, 1UL << (imm & BITMANIP_SHAMT_MASK));
        break;
    case OPC_RISC_BSETI:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
        tcg_gen_ori_tl(source1, source1, 1UL << (imm & BITMANIP_SHAMT_MASK));
        break;
#elif defined(TARGET_RISCV64)
    case OPC_RISC_CLZ:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_clzi_i64(source1, source1, 64);
        break;
    case OPC_RISC_CTZ:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, ctz64(get_gpr_raw(rs1)));
        break;
    case OPC_RISC_CPOP:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, ctpop64(get_gpr_raw(rs1)));
        break;
    case OPC_RISC_SEXT_B:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_sextract_i64(source1, source1, 0, 8);
        break;
    case OPC_RISC_SEXT_H:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_sextract_i64(source1, source1, 0, 16);
        break;
    case OPC_RISC_ORC_B:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, orcb64(get_gpr_raw(rs1)));
        break;
    case OPC_RISC_BCLRI:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
        tcg_gen_andi_tl(source1, source1, ~(1UL << (imm & BITMANIP_SHAMT_MASK)));
        break;
    case OPC_RISC_BEXTI:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
        tcg_gen_shri_tl(source1, source1, imm & BITMANIP_SHAMT_MASK);
        tcg_gen_andi_tl(source1, source1, 1UL);
        break;
    case OPC_RISC_BINVI:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
        tcg_gen_xori_tl(source1, source1, 1UL << (imm & BITMANIP_SHAMT_MASK));
        break;
    case OPC_RISC_BSETI:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBS)) {
            return;
        }
        tcg_gen_ori_tl(source1, source1, 1UL << (imm & BITMANIP_SHAMT_MASK));
        break;
#endif
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
}

static void gen_arith_imm(DisasContext *dc, uint32_t opc, int rd, int rs1, target_long imm)
{
    TCGv source1;
    source1 = tcg_temp_new();
    gen_get_gpr(source1, rs1);
    target_long extra_shamt = 0;

    switch (opc) {
    case OPC_RISC_ADDI:
#if defined(TARGET_RISCV64)
    case OPC_RISC_ADDIW:
#endif
        tcg_gen_addi_tl(source1, source1, imm);
        break;
    case OPC_RISC_SLTI:
        tcg_gen_setcondi_tl(TCG_COND_LT, source1, source1, imm);
        break;
    case OPC_RISC_SLTIU:
        tcg_gen_setcondi_tl(TCG_COND_LTU, source1, source1, imm);
        break;
    case OPC_RISC_XORI:
        tcg_gen_xori_tl(source1, source1, imm);
        break;
    case OPC_RISC_ORI:
        tcg_gen_ori_tl(source1, source1, imm);
        break;
    case OPC_RISC_ANDI:
        tcg_gen_andi_tl(source1, source1, imm);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SLLIW:
        if ((imm >= 32)) {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        /* fall through to SLLI */
#endif
    /* fallthrough */
    case OPC_RISC_SLLI:
#if defined(TARGET_RISCV32)
        if (imm >> 5) {
#elif defined(TARGET_RISCV64)
        if (imm >> 6) {
#endif
            gen_arith_bitmanip(dc, rd, rs1, imm, source1);
        } else {
            tcg_gen_shli_tl(source1, source1, imm);
        }
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_SHIFT_RIGHT_IW:
        if ((imm & 0x3ff) >= 32) {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        tcg_gen_shli_tl(source1, source1, 32);
        extra_shamt = 32;
        /* fall through to SHIFT_RIGHT_I */
#endif
    /* fallthrough */
    case OPC_RISC_SHIFT_RIGHT_I:
        /* differentiate on IMM */
#if defined(TARGET_RISCV32)
        if (imm >> 5) {
            if ((imm >> 5) == 0x20) {
#elif defined(TARGET_RISCV64)
        if (imm >> 6) {
            if ((imm >> 6) == 0x10) {
#endif
                /* SRAI[W] */
                tcg_gen_sari_tl(source1, source1, (imm ^ 0x400) + extra_shamt);
            } else {
                gen_arith_bitmanip(dc, rd, rs1, imm, source1);
            }
        } else {
            /* SRLI[W] */
            tcg_gen_shri_tl(source1, source1, imm + extra_shamt);
        }
        break;
    case OPC_RISC_SLLI_UW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBA)) {
            return;
        }
        tcg_gen_shli_tl(rd, source1, imm);
        break;
    case OPC_RISC_RORI:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
#if defined(TARGET_RISCV32)
        tcg_gen_movi_tl(source1, ror32(source1, (imm & BITMANIP_SHAMT_MASK)));
#elif defined(TARGET_RISCV64)
        tcg_gen_movi_tl(source1, ror64(source1, (imm & BITMANIP_SHAMT_MASK)));
#endif
        break;
    case OPC_RISC_RORIW:
        if (!ensure_additional_extension(dc, RISCV_FEATURE_ZBB)) {
            return;
        }
        tcg_gen_movi_tl(source1, sextract64(ror32(source1, imm), 0, 32));
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    if (opc & 0x8) { /* sign-extend for W instructions */
        tcg_gen_ext32s_tl(source1, source1);
    }

    gen_set_gpr(rd, source1);

#ifdef DL_TRACE_ARITH
    /* DataLab: helper call and free temporary variable */
    gen_sync_pc(dc);
    TCGv_i32 topc = tcg_const_i32(opc);
    TCGv_i32 topclass = tcg_const_i32(DL_RISC_ARITH_IMM);
    gen_helper_log_inst_arith(cpu_env, topc, topclass);
    tcg_temp_free_i32(topc);
    tcg_temp_free_i32(topclass);
    //==================================================//
 #endif

    tcg_temp_free(source1);
}

static inline int is_jal_an_ret_pseudoinsn(int rd, int rs1, int imm)
{
    // ret => jalr x0, 0(x1)
    return (rs1 == 1) && (rd == 0) && (imm == 0);
}

static inline int is_jal_RA_based(int rd)
{
    // jalr x1, NN(XX)
    return (rd == 1);
}

static inline void announce_if_jump_or_ret(int rd, int rs1, target_long imm, target_ulong next_pc)
{
    int type = STACK_FRAME_NO_CHANGE;

    if (is_jal_an_ret_pseudoinsn(rd, rs1, imm)) {
        type = STACK_FRAME_POP;
    } else if (is_jal_RA_based(rd)) {
        type = STACK_FRAME_ADD;
    }
 
    if (next_pc == PROFILER_TCG_PC) {
        generate_stack_announcement(cpu_pc, type, false);
    } else {
        generate_stack_announcement_imm_i64(next_pc, type, false);
    }
}

static void gen_jal(CPUState *env, DisasContext *dc, int rd, target_ulong imm)
{
    target_ulong next_pc;

    /* check misaligned: */
    next_pc = dc->base.pc + imm;

    if (!riscv_has_ext(env, RISCV_FEATURE_RVC)) {
        if ((next_pc & 0x3) != 0) {
            generate_exception_mbadaddr(dc, RISCV_EXCP_INST_ADDR_MIS);
        }
    }

    get_set_gpr_imm(rd, dc->npc);

    if (unlikely(dc->base.guest_profile)) {
        announce_if_jump_or_ret(rd, RA, imm, next_pc);
    }

    gen_goto_tb(dc, 0, dc->base.pc + imm); /* must use this for safety */
    dc->base.is_jmp = DISAS_BRANCH;

}

static void gen_jalr(CPUState *env, DisasContext *dc, uint32_t opc, int rd, int rs1, target_long imm)
{
    /* no chaining with JALR */
    int misaligned = gen_new_label();
    TCGv t0;
    t0 = tcg_temp_new();

    switch (opc) {
    case OPC_RISC_JALR:
        gen_get_gpr(cpu_pc, rs1);
        tcg_gen_addi_tl(cpu_pc, cpu_pc, imm);
        tcg_gen_andi_tl(cpu_pc, cpu_pc, (target_ulong) - 2);

        if (!riscv_has_ext(env, RISCV_FEATURE_RVC)) {
            tcg_gen_andi_tl(t0, cpu_pc, 0x2);
            tcg_gen_brcondi_tl(TCG_COND_NE, t0, 0x0, misaligned);
        }

        get_set_gpr_imm(rd, dc->npc);
        if (unlikely(dc->base.guest_profile)) {
            announce_if_jump_or_ret(rd, rs1, imm, PROFILER_TCG_PC);
        }

        gen_exit_tb_no_chaining(dc->base.tb);

        gen_set_label(misaligned);
        generate_exception_mbadaddr(dc, RISCV_EXCP_INST_ADDR_MIS);
        gen_exit_tb_no_chaining(dc->base.tb);
        dc->base.is_jmp = DISAS_BRANCH;
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free(t0);
}

static void gen_branch(CPUState *env, DisasContext *dc, uint32_t opc, int rs1, int rs2, target_long bimm)
{
    int l = gen_new_label();
    TCGv source1, source2;
    source1 = tcg_temp_new();
    source2 = tcg_temp_new();
    gen_get_gpr(source1, rs1);
    gen_get_gpr(source2, rs2);

    switch (opc) {
    case OPC_RISC_BEQ:
        tcg_gen_brcond_tl(TCG_COND_EQ, source1, source2, l);
        break;
    case OPC_RISC_BNE:
        tcg_gen_brcond_tl(TCG_COND_NE, source1, source2, l);
        break;
    case OPC_RISC_BLT:
        tcg_gen_brcond_tl(TCG_COND_LT, source1, source2, l);
        break;
    case OPC_RISC_BGE:
        tcg_gen_brcond_tl(TCG_COND_GE, source1, source2, l);
        break;
    case OPC_RISC_BLTU:
        tcg_gen_brcond_tl(TCG_COND_LTU, source1, source2, l);
        break;
    case OPC_RISC_BGEU:
        tcg_gen_brcond_tl(TCG_COND_GEU, source1, source2, l);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    gen_goto_tb(dc, 1, dc->npc);
    gen_set_label(l); /* branch taken */
    if (!riscv_has_ext(env, RISCV_FEATURE_RVC) && ((dc->base.pc + bimm) & 0x3)) {
        /* misaligned */
        generate_exception_mbadaddr(dc, RISCV_EXCP_INST_ADDR_MIS);
        gen_exit_tb_no_chaining(dc->base.tb);
    } else {
        gen_goto_tb(dc, 0, dc->base.pc + bimm);
    }
    tcg_temp_free(source1);
    tcg_temp_free(source2);
    dc->base.is_jmp = DISAS_BRANCH;
}

static void gen_load(DisasContext *dc, uint32_t opc, int rd, int rs1, target_long imm)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();

    gen_get_gpr(t0, rs1);

    tcg_gen_addi_tl(t0, t0, imm);

    gen_sync_pc(dc);

    /* DataLab: helper call */
    TCGv_i32 topc = tcg_const_i32(opc);
    gen_helper_log_inst(cpu_env, topc, t0);

    switch (opc) {

    case OPC_RISC_LB:
        tcg_gen_qemu_ld8s(t1, t0, dc->base.mem_idx);
        break;
    case OPC_RISC_LH:
        tcg_gen_qemu_ld16s(t1, t0, dc->base.mem_idx);
        break;
    case OPC_RISC_LW:
        tcg_gen_qemu_ld32s(t1, t0, dc->base.mem_idx);
        break;
    case OPC_RISC_LD:
        tcg_gen_qemu_ld64(t1, t0, dc->base.mem_idx);
        break;
    case OPC_RISC_LBU:
        tcg_gen_qemu_ld8u(t1, t0, dc->base.mem_idx);
        break;
    case OPC_RISC_LHU:
        tcg_gen_qemu_ld16u(t1, t0, dc->base.mem_idx);
        break;
    case OPC_RISC_LWU:
        tcg_gen_qemu_ld32u(t1, t0, dc->base.mem_idx);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;

    }

    gen_set_gpr(rd, t1);

    /* DataLab: free temporary variable */
    tcg_temp_free_i32(topc);
    //====================================

    tcg_temp_free(t0);
    tcg_temp_free(t1);
}

static void gen_store(DisasContext *dc, uint32_t opc, int rs1, int rs2, target_long imm)
{
    gen_sync_pc(dc);

    TCGv t0 = tcg_temp_new();
    TCGv dat = tcg_temp_new();
    gen_get_gpr(t0, rs1);
    tcg_gen_addi_tl(t0, t0, imm);
    gen_get_gpr(dat, rs2);

    /* DataLab: helper call */
    TCGv_i32 topc = tcg_const_i32(opc);
    gen_helper_log_inst(cpu_env, topc, t0);

    switch (opc) {
    case OPC_RISC_SB:
        tcg_gen_qemu_st8(dat, t0, dc->base.mem_idx);
        break;
    case OPC_RISC_SH:
        tcg_gen_qemu_st16(dat, t0, dc->base.mem_idx);
        break;
    case OPC_RISC_SW:
        tcg_gen_qemu_st32(dat, t0, dc->base.mem_idx);
        break;
    case OPC_RISC_SD:
        tcg_gen_qemu_st64(dat, t0, dc->base.mem_idx);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    /* DataLab: free temporary variable */
    tcg_temp_free_i32(topc);
    //====================================

    tcg_temp_free(t0);
    tcg_temp_free(dat);
}

static void gen_fp_load(DisasContext *dc, uint32_t opc, int rd, int rs1, target_long imm)
{
    if (!ensure_fp_extension_for_load_store(dc)) {
        return;
    }

    TCGv t0 = tcg_temp_local_new();
    int fp_ok = gen_new_label();
    int done = gen_new_label();

    // check MSTATUS.FS
    tcg_gen_ld_tl(t0, cpu_env, offsetof(CPUState, mstatus));
    tcg_gen_andi_tl(t0, t0, MSTATUS_FS);
    tcg_gen_brcondi_tl(TCG_COND_NE, t0, 0x0, fp_ok);
    // MSTATUS_FS field was zero:
    kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
    tcg_gen_br(done);

    // proceed with operation
    gen_set_label(fp_ok);
    gen_get_gpr(t0, rs1);
    tcg_gen_addi_tl(t0, t0, imm);

    /* DataLab: helper call */
    gen_sync_pc(dc);
    TCGv_i32 topc = tcg_const_i32(opc);
    gen_helper_log_inst(cpu_env, topc, t0);

    TCGv destination = tcg_temp_new();
    switch (opc) {
    case OPC_RISC_FLH:
        tcg_gen_qemu_ld16u(destination, t0, dc->base.mem_idx);
        tcg_gen_extu_tl_i64(cpu_fpr[rd], destination);
        gen_box_float(RISCV_HALF_PRECISION, cpu_fpr[rd]);
        break;
    case OPC_RISC_FLW:
        tcg_gen_qemu_ld32u(destination, t0, dc->base.mem_idx);
        tcg_gen_extu_tl_i64(cpu_fpr[rd], destination);
        gen_box_float(RISCV_SINGLE_PRECISION, cpu_fpr[rd]);
        break;
    case OPC_RISC_FLD:
        tcg_gen_qemu_ld64(cpu_fpr[rd], t0, dc->base.mem_idx);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free(destination);
    gen_set_label(done);
    tcg_temp_free(t0);

    /* DataLab: free temporary variable */
    tcg_temp_free_i32(topc);
    //====================================
}

static void gen_v_load(DisasContext *dc, uint32_t opc, uint32_t rest, uint32_t vd, uint32_t rs1, uint32_t rs2, uint32_t width)
{
// Vector helpers require 128-bit ints which aren't supported on 32-bit hosts.
#if HOST_LONG_BITS == 32
    tlib_abort("Vector extension isn't available on 32-bit hosts.");
#else
    uint32_t vm = extract32(rest, 0, 1);
    uint32_t mew = extract32(rest, 3, 1);
    uint32_t nf = extract32(rest, 4, 3);
    if (!ensure_extension(dc, RISCV_FEATURE_RVV) || mew) {
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        return;
    }
    if (MASK_OP_V_LOAD_US(dc->opcode) != OPC_RISC_VL_US_WR) {
        generate_vill_check(dc);
    }
    TCGv_i32 t_vd, t_rs1, t_rs2, t_nf;
    t_vd = tcg_temp_new_i32();
    t_rs1 = tcg_temp_new_i32();
    t_rs2 = tcg_temp_new_i32();
    t_nf = tcg_temp_new_i32();
    tcg_gen_movi_i32(t_vd, vd);
    tcg_gen_movi_i32(t_rs1, rs1);
    tcg_gen_movi_i32(t_rs2, rs2);
    tcg_gen_movi_i32(t_nf, nf);

    switch (opc) {
    case OPC_RISC_VL_US: // unit-stride
        switch (MASK_OP_V_LOAD_US(dc->opcode)) {
        case OPC_RISC_VL_US:
            switch (width & 0x3) {
            case 0:
                if (vm) {
                    gen_helper_vle8(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vle8_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            case 1:
                if (vm) {
                    gen_helper_vle16(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vle16_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            case 2:
                if (vm) {
                    gen_helper_vle32(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vle32_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            case 3:
                if (vm) {
                    gen_helper_vle64(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vle64_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            }
            break;
        case OPC_RISC_VL_US_WR:
            if (!vm || !is_nfields_power_of_two(nf)) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            }
            gen_helper_vl_wr(cpu_env, t_vd, t_rs1, t_nf);
            break;
        case OPC_RISC_VL_US_MASK:
            if (!vm || width || nf) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            }
            gen_helper_vlm(cpu_env, t_vd, t_rs1);
            break;
        case OPC_RISC_VL_US_FOF:
            switch (width & 0x3) {
            case 0:
                if (vm) {
                    gen_helper_vle8ff(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vle8ff_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            case 1:
                if (vm) {
                    gen_helper_vle16ff(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vle16ff_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            case 2:
                if (vm) {
                    gen_helper_vle32ff(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vle32ff_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            case 3:
                if (vm) {
                    gen_helper_vle64ff(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vle64ff_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            }
            break;
        }
        break;
    case OPC_RISC_VL_VS: // vector-strided
        switch (width & 0x3) {
        case 0:
            if (vm) {
                gen_helper_vlse8(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vlse8_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 1:
            if (vm) {
                gen_helper_vlse16(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vlse16_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 2:
            if (vm) {
                gen_helper_vlse32(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vlse32_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 3:
            if (vm) {
                gen_helper_vlse64(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vlse64_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        }
        break;
    case OPC_RISC_VL_UVI: // unordered vector-indexed
    case OPC_RISC_VL_OVI: // ordered vector-indexed
        switch (width & 0x3) {
        case 0:
            if (vm) {
                gen_helper_vlxei8(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vlxei8_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 1:
            if (vm) {
                gen_helper_vlxei16(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vlxei16_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 2:
            if (vm) {
                gen_helper_vlxei32(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vlxei32_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 3:
            if (vm) {
                gen_helper_vlxei64(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vlxei64_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        }
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_gen_movi_tl(cpu_vstart, 0);

    /* DataLab: helper call */
    gen_sync_pc(dc);
    TCGv_i32 topc = tcg_const_i32(opc);
    TCGv_i32 twidth = tcg_const_i32(width);
    TCGv taddr = tcg_temp_new();
    gen_get_gpr(taddr, rs1);
    //gen_helper_log_inst_vector(cpu_env, topc, t_rs1, twidth);
    gen_helper_log_inst_vector(cpu_env, topc, taddr, twidth);
    //========================================================

    tcg_temp_free_i32(t_vd);
    tcg_temp_free_i32(t_rs1);
    tcg_temp_free_i32(t_rs2);
    tcg_temp_free_i32(t_nf);

    /* DataLab: free temporary variable */
    tcg_temp_free_i32(topc);
    tcg_temp_free_i32(twidth);
    tcg_temp_free(taddr);
    //====================================
#endif  // HOST_LONG_BITS != 32
}

static void gen_fp_store(DisasContext *dc, uint32_t opc, int rs1, int rs2, target_long imm)
{
    if (!ensure_fp_extension_for_load_store(dc)) {
        return;
    }

    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    int fp_ok = gen_new_label();
    int done = gen_new_label();

    // check MSTATUS.FS
    tcg_gen_ld_tl(t0, cpu_env, offsetof(CPUState, mstatus));
    tcg_gen_andi_tl(t0, t0, MSTATUS_FS);
    tcg_gen_brcondi_tl(TCG_COND_NE, t0, 0x0, fp_ok);
    // MSTATUS_FS field was zero:
    kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
    tcg_gen_br(done);

    // proceed with operation
    gen_set_label(fp_ok);
    gen_get_gpr(t0, rs1);
    tcg_gen_addi_tl(t0, t0, imm);

    /* DataLab: helper call */
    TCGv_i32 topc = tcg_const_i32(opc);
    gen_helper_log_inst(cpu_env, topc, t0);

    switch (opc) {
    case OPC_RISC_FSH:
        tcg_gen_qemu_st16(cpu_fpr[rs2], t0, dc->base.mem_idx);
        break;
    case OPC_RISC_FSW:
        tcg_gen_qemu_st32(cpu_fpr[rs2], t0, dc->base.mem_idx);
        break;
    case OPC_RISC_FSD:
        tcg_gen_qemu_st64(cpu_fpr[rs2], t0, dc->base.mem_idx);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    gen_set_label(done);
    tcg_temp_free(t0);
    tcg_temp_free(t1);

    /* DataLab: free temporary variable */
    tcg_temp_free_i32(topc);
    //====================================
}

static void gen_v_store(DisasContext *dc, uint32_t opc, uint32_t rest, uint32_t vd, uint32_t rs1, uint32_t rs2, uint32_t width)
{
// Vector helpers require 128-bit ints which aren't supported on 32-bit hosts.
#if HOST_LONG_BITS == 32
    tlib_abort("Vector extension isn't available on 32-bit hosts.");
#else
    uint32_t vm = extract32(rest, 0, 1);
    uint32_t mew = extract32(rest, 3, 1);
    uint32_t nf = extract32(rest, 4, 3);
    if (!ensure_extension(dc, RISCV_FEATURE_RVV) || mew) {
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        return;
    }
    if (MASK_OP_V_STORE_US(dc->opcode) != OPC_RISC_VS_US_WR) {
        generate_vill_check(dc);
    }
    TCGv_i32 t_vd, t_rs1, t_rs2, t_nf;
    t_vd = tcg_temp_new_i32();
    t_rs1 = tcg_temp_new_i32();
    t_rs2 = tcg_temp_new_i32();
    t_nf = tcg_temp_new_i32();
    tcg_gen_movi_i32(t_vd, vd);
    tcg_gen_movi_i32(t_rs1, rs1);
    tcg_gen_movi_i32(t_rs2, rs2);
    tcg_gen_movi_i32(t_nf, nf);


    switch (opc) {
    case OPC_RISC_VS_US: // unit-stride
        switch (MASK_OP_V_STORE_US(dc->opcode)) {
        case OPC_RISC_VS_US:
            switch (width & 0x3) {
            case 0:
                if (vm) {
                    gen_helper_vse8(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vse8_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            case 1:
                if (vm) {
                    gen_helper_vse16(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vse16_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            case 2:
                if (vm) {
                    gen_helper_vse32(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vse32_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            case 3:
                if (vm) {
                    gen_helper_vse64(cpu_env, t_vd, t_rs1, t_nf);
                } else {
                    gen_helper_vse64_m(cpu_env, t_vd, t_rs1, t_nf);
                }
                break;
            }
            break;
        case OPC_RISC_VS_US_WR:
            if (!vm || width || !is_nfields_power_of_two(nf)) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            }
            gen_helper_vs_wr(cpu_env, t_vd, t_rs1, t_nf);
            break;
        case OPC_RISC_VS_US_MASK:
            if (!vm || width || nf) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            }
            gen_helper_vsm(cpu_env, t_vd, t_rs1);
            break;
        default:
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        break;
    case OPC_RISC_VS_VS: // vector-strided
        switch (width & 0x3) {
        case 0:
            if (vm) {
                gen_helper_vsse8(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vsse8_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 1:
            if (vm) {
                gen_helper_vsse16(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vsse16_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 2:
            if (vm) {
                gen_helper_vsse32(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vsse32_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 3:
            if (vm) {
                gen_helper_vsse64(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vsse64_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        }
        break;
    case OPC_RISC_VS_UVI: // unordered vector-indexed
    case OPC_RISC_VS_OVI: // ordered vector-indexed
        switch (width & 0x3) {
        case 0:
            if (vm) {
                gen_helper_vsxei8(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vsxei8_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 1:
            if (vm) {
                gen_helper_vsxei16(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vsxei16_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 2:
            if (vm) {
                gen_helper_vsxei32(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vsxei32_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        case 3:
            if (vm) {
                gen_helper_vsxei64(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            } else {
                gen_helper_vsxei64_m(cpu_env, t_vd, t_rs1, t_rs2, t_nf);
            }
            break;
        }
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_gen_movi_tl(cpu_vstart, 0);

    /* DataLab: helper call */
    gen_sync_pc(dc);
    TCGv_i32 topc = tcg_const_i32(opc);
    TCGv_i32 twidth = tcg_const_i32(width);
    TCGv taddr = tcg_temp_new();
    gen_get_gpr(taddr, rs1);
    //gen_helper_log_inst_vector(cpu_env, topc, t_rs1, twidth);
    gen_helper_log_inst_vector(cpu_env, topc, taddr, twidth);
    //=============================================================

    tcg_temp_free_i32(t_vd);
    tcg_temp_free_i32(t_rs1);
    tcg_temp_free_i32(t_rs2);
    tcg_temp_free_i32(t_nf);

    /* DataLab: free temporary variable */
    tcg_temp_free_i32(topc);
    tcg_temp_free_i32(twidth);
    tcg_temp_free(taddr);
    //====================================
#endif  // HOST_LONG_BITS != 32
}

static void gen_atomic(CPUState *env, DisasContext *dc, uint32_t opc, int rd, int rs1, int rs2)
{
    if (!ensure_extension(dc, RISCV_FEATURE_RVA)) {
        return;
    }

    /* TODO: handle aq, rl bits? - for now just get rid of them: */
    opc = MASK_OP_ATOMIC_NO_AQ_RL(opc);
    TCGv source1, source2, dat;
    int done;
    int finish_label;
    source1 = tcg_temp_local_new();
    source2 = tcg_temp_local_new();
    done = gen_new_label();
    dat = tcg_temp_local_new();
    gen_get_gpr(source1, rs1);
    gen_get_gpr(source2, rs2);

    gen_sync_pc(dc);

    gen_helper_acquire_global_memory_lock(cpu_env);

    switch (opc) {
    case OPC_RISC_LR_W:
        gen_helper_reserve_address(cpu_env, source1, tcg_const_i32(0));
        tcg_gen_qemu_ld32s(dat, source1, dc->base.mem_idx);
        break;
    case OPC_RISC_SC_W:
        finish_label = gen_new_label();
        gen_helper_check_address_reservation(dat, cpu_env, source1);
        tcg_gen_brcondi_tl(TCG_COND_NE, dat, 0, finish_label);
        tcg_gen_qemu_st32(source2, source1, dc->base.mem_idx);
        gen_set_label(finish_label);
        break;
    case OPC_RISC_AMOSWAP_W:
        tcg_gen_qemu_ld32s(dat, source1, dc->base.mem_idx);
        tcg_gen_qemu_st32(source2, source1, dc->base.mem_idx);
        tcg_gen_mov_tl(source1, dat);
        break;
    case OPC_RISC_AMOADD_W:
        tcg_gen_qemu_ld32s(dat, source1, dc->base.mem_idx);
        tcg_gen_add_tl(source2, dat, source2);
        tcg_gen_qemu_st32(source2, source1, dc->base.mem_idx);
        tcg_gen_mov_tl(source1, dat);
        break;
    case OPC_RISC_AMOXOR_W:
        tcg_gen_qemu_ld32s(dat, source1, dc->base.mem_idx);
        tcg_gen_xor_tl(source2, dat, source2);
        tcg_gen_qemu_st32(source2, source1, dc->base.mem_idx);
        tcg_gen_mov_tl(source1, dat);
        break;
    case OPC_RISC_AMOAND_W:
        tcg_gen_qemu_ld32s(dat, source1, dc->base.mem_idx);
        tcg_gen_and_tl(source2, dat, source2);
        tcg_gen_qemu_st32(source2, source1, dc->base.mem_idx);
        tcg_gen_mov_tl(source1, dat);
        break;
    case OPC_RISC_AMOOR_W:
        tcg_gen_qemu_ld32s(dat, source1, dc->base.mem_idx);
        tcg_gen_or_tl(source2, dat, source2);
        tcg_gen_qemu_st32(source2, source1, dc->base.mem_idx);
        tcg_gen_mov_tl(source1, dat);
        break;
    case OPC_RISC_AMOMIN_W:
        tcg_gen_qemu_ld32s(dat, source1, dc->base.mem_idx);
        tcg_gen_brcond_i32(TCG_COND_LT, dat, source2, done);
        tcg_gen_qemu_st32(source2, source1, dc->base.mem_idx);
        break;
    case OPC_RISC_AMOMAX_W:
        tcg_gen_qemu_ld32s(dat, source1, dc->base.mem_idx);
        tcg_gen_brcond_i32(TCG_COND_GT, dat, source2, done);
        tcg_gen_qemu_st32(source2, source1, dc->base.mem_idx);
        break;
    case OPC_RISC_AMOMINU_W:
        tcg_gen_qemu_ld32s(dat, source1, dc->base.mem_idx);
        tcg_gen_brcond_i32(TCG_COND_LTU, dat, source2, done);
        tcg_gen_qemu_st32(source2, source1, dc->base.mem_idx);
        break;
    case OPC_RISC_AMOMAXU_W:
        tcg_gen_qemu_ld32s(dat, source1, dc->base.mem_idx);
        tcg_gen_brcond_i32(TCG_COND_GTU, dat, source2, done);
        tcg_gen_qemu_st32(source2, source1, dc->base.mem_idx);
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_LR_D:
        tcg_gen_qemu_ld64(dat, source1, dc->base.mem_idx);
        break;
    case OPC_RISC_SC_D:
        tcg_gen_qemu_st64(source2, source1, dc->base.mem_idx);
        tcg_gen_movi_tl(dat, 0); // assume always success
        break;
    case OPC_RISC_AMOSWAP_D:
        tcg_gen_qemu_ld64(dat, source1, dc->base.mem_idx);
        tcg_gen_qemu_st64(source2, source1, dc->base.mem_idx);
        tcg_gen_mov_tl(source1, dat);
        break;
    case OPC_RISC_AMOADD_D:
        tcg_gen_qemu_ld64(dat, source1, dc->base.mem_idx);
        tcg_gen_add_tl(source2, dat, source2);
        tcg_gen_qemu_st64(source2, source1, dc->base.mem_idx);
        tcg_gen_mov_tl(source1, dat);
        break;
    case OPC_RISC_AMOXOR_D:
        tcg_gen_qemu_ld64(dat, source1, dc->base.mem_idx);
        tcg_gen_xor_tl(source2, dat, source2);
        tcg_gen_qemu_st64(source2, source1, dc->base.mem_idx);
        tcg_gen_mov_tl(source1, dat);
        break;
    case OPC_RISC_AMOAND_D:
        tcg_gen_qemu_ld64(dat, source1, dc->base.mem_idx);
        tcg_gen_and_tl(source2, dat, source2);
        tcg_gen_qemu_st64(source2, source1, dc->base.mem_idx);
        tcg_gen_mov_tl(source1, dat);
        break;
    case OPC_RISC_AMOOR_D:
        tcg_gen_qemu_ld64(dat, source1, dc->base.mem_idx);
        tcg_gen_or_tl(source2, dat, source2);
        tcg_gen_qemu_st64(source2, source1, dc->base.mem_idx);
        tcg_gen_mov_tl(source1, dat);
        break;
    case OPC_RISC_AMOMIN_D:
        tcg_gen_qemu_ld64(dat, source1, dc->base.mem_idx);
        tcg_gen_brcond_tl(TCG_COND_LT, dat, source2, done);
        tcg_gen_qemu_st64(source2, source1, dc->base.mem_idx);
        break;
    case OPC_RISC_AMOMAX_D:
        tcg_gen_qemu_ld64(dat, source1, dc->base.mem_idx);
        tcg_gen_brcond_tl(TCG_COND_GT, dat, source2, done);
        tcg_gen_qemu_st64(source2, source1, dc->base.mem_idx);
        break;
    case OPC_RISC_AMOMINU_D:
        tcg_gen_qemu_ld64(dat, source1, dc->base.mem_idx);
        tcg_gen_brcond_tl(TCG_COND_LTU, dat, source2, done);
        tcg_gen_qemu_st64(source2, source1, dc->base.mem_idx);
        break;
    case OPC_RISC_AMOMAXU_D:
        tcg_gen_qemu_ld64(dat, source1, dc->base.mem_idx);
        tcg_gen_brcond_tl(TCG_COND_GTU, dat, source2, done);
        tcg_gen_qemu_st64(source2, source1, dc->base.mem_idx);
        break;
#endif
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

    gen_helper_release_global_memory_lock(cpu_env);

    gen_set_label(done);
    gen_set_gpr(rd, dat);
    tcg_temp_free(source1);
    tcg_temp_free(source2);
    tcg_temp_free(dat);
}

static void gen_fp_helper_fpr_3fpr_1imm(void (*gen_fp_helper)(TCGv_i64, TCGv_ptr, TCGv_i64, TCGv_i64, TCGv_i64, TCGv_i64), enum riscv_floating_point_precision float_precision, int rd, int rs1, int rs2, int rs3, uint64_t rm)
{
    TCGv_i64 rs1_boxed = tcg_temp_local_new_i64();
    TCGv_i64 rs2_boxed = tcg_temp_local_new_i64();
    TCGv_i64 rs3_boxed = tcg_temp_local_new_i64();
    gen_unbox_float(float_precision, env, rs1_boxed, cpu_fpr[rs1]);
    gen_unbox_float(float_precision, env, rs2_boxed, cpu_fpr[rs2]);
    gen_unbox_float(float_precision, env, rs3_boxed, cpu_fpr[rs3]);

    TCGv_i64 rm_reg = tcg_temp_new_i64();
    tcg_gen_movi_i64(rm_reg, rm);

    gen_fp_helper(cpu_fpr[rd], cpu_env, rs1_boxed, rs2_boxed, rs3_boxed, rm_reg);
    gen_box_float(float_precision, cpu_fpr[rd]);

    tcg_temp_free_i64(rm_reg);
    tcg_temp_free_i64(rs1_boxed);
    tcg_temp_free_i64(rs2_boxed);
    tcg_temp_free_i64(rs3_boxed);
}

static void gen_fp_helper_fpr_2fpr_1tcg(void (*gen_fp_helper)(TCGv_i64, TCGv_ptr, TCGv_i64, TCGv_i64, TCGv_i64), enum riscv_floating_point_precision float_precision, int rd, int rs1, int rs2, TCGv_i64 rm_reg)
{
    TCGv_i64 rs1_boxed = tcg_temp_local_new_i64();
    TCGv_i64 rs2_boxed = tcg_temp_local_new_i64();
    gen_unbox_float(float_precision, env, rs1_boxed, cpu_fpr[rs1]);
    gen_unbox_float(float_precision, env, rs2_boxed, cpu_fpr[rs2]);

    gen_fp_helper(cpu_fpr[rd], cpu_env, rs1_boxed, rs2_boxed, rm_reg);

    gen_box_float(float_precision, cpu_fpr[rd]);
    tcg_temp_free_i64(rs1_boxed);
    tcg_temp_free_i64(rs2_boxed);
}

static void gen_fp_helper_fpr_2fpr(void (*gen_fp_helper)(TCGv_i64, TCGv_ptr, TCGv_i64, TCGv_i64), enum riscv_floating_point_precision float_precision, int rd, int rs1, int rs2)
{
    TCGv_i64 rs1_boxed = tcg_temp_local_new_i64();
    TCGv_i64 rs2_boxed = tcg_temp_local_new_i64();
    gen_unbox_float(float_precision, env, rs1_boxed, cpu_fpr[rs1]);
    gen_unbox_float(float_precision, env, rs2_boxed, cpu_fpr[rs2]);

    gen_fp_helper(cpu_fpr[rd], cpu_env, rs1_boxed, rs2_boxed);

    gen_box_float(float_precision, cpu_fpr[rd]);
    tcg_temp_free_i64(rs1_boxed);
    tcg_temp_free_i64(rs2_boxed);
}

static void gen_fp_helper_fpr_1fpr_1tcg(void (*gen_fp_helper)(TCGv_i64, TCGv_ptr, TCGv_i64, TCGv_i64), enum riscv_floating_point_precision float_precision, int rd, int rs1, TCGv_i64 rm_reg)
{
    TCGv_i64 rs1_boxed = tcg_temp_local_new_i64();
    gen_unbox_float(float_precision, env, rs1_boxed, cpu_fpr[rs1]);

    gen_fp_helper(cpu_fpr[rd], cpu_env, rs1_boxed, rm_reg);

    gen_box_float(float_precision, cpu_fpr[rd]);
    tcg_temp_free_i64(rs1_boxed);
}

static void gen_fp_helper_gpr_2fpr(void (*gen_fp_helper)(TCGv_i64, TCGv_ptr, TCGv_i64, TCGv_i64), enum riscv_floating_point_precision float_precision, TCGv_i64 rd_reg, int rd, int rs1, int rs2)
{
    TCGv_i64 rs1_boxed = tcg_temp_local_new_i64();
    TCGv_i64 rs2_boxed = tcg_temp_local_new_i64();
    gen_unbox_float(float_precision, env, rs1_boxed, cpu_fpr[rs1]);
    gen_unbox_float(float_precision, env, rs2_boxed, cpu_fpr[rs2]);

    gen_fp_helper(rd_reg, cpu_env, rs1_boxed, rs2_boxed);

    gen_set_gpr(rd, rd_reg);
    tcg_temp_free_i64(rs1_boxed);
    tcg_temp_free_i64(rs2_boxed);
}

static void gen_fp_helper_gpr_1fpr_1tcg(void (*gen_fp_helper)(TCGv_i64, TCGv_ptr, TCGv_i64, TCGv_i64), enum riscv_floating_point_precision float_precision, TCGv_i64 rd_reg, int rd, int rs1, TCGv_i64 rm_reg)
{
    TCGv_i64 rs1_boxed = tcg_temp_local_new_i64();
    gen_unbox_float(float_precision, env, rs1_boxed, cpu_fpr[rs1]);

    gen_fp_helper(rd_reg, cpu_env, rs1_boxed, rm_reg);

    gen_set_gpr(rd, rd_reg);
    tcg_temp_free_i64(rs1_boxed);
}

static void gen_fp_helper_fpr_1gpr_1tcg(void (*gen_fp_helper)(TCGv_i64, TCGv_ptr, TCGv_i64, TCGv_i64), enum riscv_floating_point_precision float_precision, TCGv_i64 tmp_reg, int rd, int rs1, TCGv_i64 rm_reg)
{
    gen_get_gpr(tmp_reg, rs1);

    gen_fp_helper(cpu_fpr[rd], cpu_env, tmp_reg, rm_reg);

    gen_box_float(float_precision, cpu_fpr[rd]);
}

static void gen_fp_fmadd(DisasContext* dc, uint32_t opc, int rd, int rs1, int rs2, int rs3, int rm)
{
    if (!ensure_fp_extension(dc, 25)) {
        return;
    }

    switch (opc) {
    case OPC_RISC_FMADD_S:
        gen_fp_helper_fpr_3fpr_1imm(gen_helper_fmadd_s, RISCV_SINGLE_PRECISION, rd, rs1, rs2, rs3, rm);
        break;
    case OPC_RISC_FMADD_D: {
        TCGv_i64 rm_reg = tcg_temp_new_i64();
        tcg_gen_movi_i64(rm_reg, rm);
        gen_helper_fmadd_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2], cpu_fpr[rs3], rm_reg);
        tcg_temp_free_i64(rm_reg);
        break;
    }
    case OPC_RISC_FMADD_H:
        gen_fp_helper_fpr_3fpr_1imm(gen_helper_fmadd_h, RISCV_HALF_PRECISION, rd, rs1, rs2, rs3, rm);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

#ifdef DL_TRACE_ARITH
    // DataLab: helper call
    gen_sync_pc(dc);
    TCGv_i32 topc = tcg_const_i32(opc);
    TCGv_i32 topclass = tcg_const_i32(DL_RISC_FMADD);
    gen_helper_log_inst_arith(cpu_env, topc, topclass);

    // DataLab: free temporary variables
    tcg_temp_free_i32(topc);
    tcg_temp_free_i32(topclass);
#endif
}

static void gen_fp_fmsub(DisasContext *dc, uint32_t opc, int rd, int rs1, int rs2, int rs3, int rm)
{
    if (!ensure_fp_extension(dc, 25)) {
        return;
    }

    switch (opc) {
    case OPC_RISC_FMSUB_S:
        gen_fp_helper_fpr_3fpr_1imm(gen_helper_fmsub_s, RISCV_SINGLE_PRECISION, rd, rs1, rs2, rs3, rm);
        break;
    case OPC_RISC_FMSUB_D: {
        TCGv_i64 rm_reg = tcg_temp_new_i64();
        tcg_gen_movi_i64(rm_reg, rm);
        gen_helper_fmsub_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2], cpu_fpr[rs3], rm_reg);
        tcg_temp_free_i64(rm_reg);
        break;
    }
    case OPC_RISC_FMSUB_H:
        gen_fp_helper_fpr_3fpr_1imm(gen_helper_fmsub_h, RISCV_HALF_PRECISION, rd, rs1, rs2, rs3, rm);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

#ifdef DL_TRACE_ARITH
    // DataLab: helper call
    gen_sync_pc(dc);
    TCGv_i32 topc = tcg_const_i32(opc);
    TCGv_i32 topclass = tcg_const_i32(DL_RISC_FMSUB);
    gen_helper_log_inst_arith(cpu_env, topc, topclass);

    // DataLab: free temporary variables
    tcg_temp_free_i32(topc);
    tcg_temp_free_i32(topclass);
#endif
}

static void gen_fp_fnmsub(DisasContext *dc, uint32_t opc, int rd, int rs1, int rs2, int rs3, int rm)
{
    if (!ensure_fp_extension(dc, 25)) {
        return;
    }

    switch (opc) {
    case OPC_RISC_FNMSUB_S:
        gen_fp_helper_fpr_3fpr_1imm(gen_helper_fnmsub_s, RISCV_SINGLE_PRECISION, rd, rs1, rs2, rs3, rm);
        break;
    case OPC_RISC_FNMSUB_D: {
        TCGv_i64 rm_reg = tcg_temp_new_i64();
        tcg_gen_movi_i64(rm_reg, rm);
        gen_helper_fnmsub_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2], cpu_fpr[rs3], rm_reg);
        tcg_temp_free_i64(rm_reg);
        break;
    }
    case OPC_RISC_FNMSUB_H:
        gen_fp_helper_fpr_3fpr_1imm(gen_helper_fnmsub_h, RISCV_HALF_PRECISION, rd, rs1, rs2, rs3, rm);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

#ifdef DL_TRACE_ARITH
    // DataLab: helper call
    gen_sync_pc(dc);
    TCGv_i32 topc = tcg_const_i32(opc);
    TCGv_i32 topclass = tcg_const_i32(DL_RISC_FNMSUB);
    gen_helper_log_inst_arith(cpu_env, topc, topclass);

    // DataLab: free temporary variables
    tcg_temp_free_i32(topc);
    tcg_temp_free_i32(topclass);
#endif
}

static void gen_fp_fnmadd(DisasContext *dc, uint32_t opc, int rd, int rs1, int rs2, int rs3, int rm)
{
    if (!ensure_fp_extension(dc, 25)) {
        return;
    }

    switch (opc) {
    case OPC_RISC_FNMADD_S:
        gen_fp_helper_fpr_3fpr_1imm(gen_helper_fnmadd_s, RISCV_SINGLE_PRECISION, rd, rs1, rs2, rs3, rm);
        break;
    case OPC_RISC_FNMADD_D: {
        TCGv_i64 rm_reg = tcg_temp_new_i64();
        tcg_gen_movi_i64(rm_reg, rm);
        gen_helper_fnmadd_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2], cpu_fpr[rs3], rm_reg);
        tcg_temp_free_i64(rm_reg);
        break;
    }
    case OPC_RISC_FNMADD_H:
        gen_fp_helper_fpr_3fpr_1imm(gen_helper_fnmadd_h, RISCV_HALF_PRECISION, rd, rs1, rs2, rs3, rm);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

#ifdef DL_TRACE_ARITH
    // DataLab: helper call
    gen_sync_pc(dc);
    TCGv_i32 topc = tcg_const_i32(opc);
    TCGv_i32 topclass = tcg_const_i32(DL_RISC_FNMADD);
    gen_helper_log_inst_arith(cpu_env, topc, topclass);

    // DataLab: free temporary variables
    tcg_temp_free_i32(topc);
    tcg_temp_free_i32(topclass);
#endif
}

static void gen_fp_arith(DisasContext *dc, uint32_t opc, int rd, int rs1, int rs2, int rm)
{
    if (!ensure_fp_extension(dc, 25)) {
        return;
    }

    TCGv_i64 rm_reg = tcg_temp_local_new_i64();
    TCGv write_int_rd = tcg_temp_local_new();
    tcg_gen_movi_i64(rm_reg, rm);
    switch (opc) {
    case OPC_RISC_FADD_S:
        gen_fp_helper_fpr_2fpr_1tcg(gen_helper_fadd_s, RISCV_SINGLE_PRECISION, rd, rs1, rs2, rm_reg);
        break;
    case OPC_RISC_FSUB_S:
        gen_fp_helper_fpr_2fpr_1tcg(gen_helper_fsub_s, RISCV_SINGLE_PRECISION, rd, rs1, rs2, rm_reg);
        break;
    case OPC_RISC_FMUL_S:
        gen_fp_helper_fpr_2fpr_1tcg(gen_helper_fmul_s, RISCV_SINGLE_PRECISION, rd, rs1, rs2, rm_reg);
        break;
    case OPC_RISC_FDIV_S:
        gen_fp_helper_fpr_2fpr_1tcg(gen_helper_fdiv_s, RISCV_SINGLE_PRECISION, rd, rs1, rs2, rm_reg);
        break;
    case OPC_RISC_FSGNJ_S:
        gen_fsgnj(dc, rd, rs1, rs2, rm, RISCV_SINGLE_PRECISION);
        break;
    case OPC_RISC_FMIN_S:
        /* also handles: OPC_RISC_FMAX_S */
        if (rm == 0x0) {
            gen_fp_helper_fpr_2fpr(gen_helper_fmin_s, RISCV_SINGLE_PRECISION, rd, rs1, rs2);
        } else if (rm == 0x1) {
            gen_fp_helper_fpr_2fpr(gen_helper_fmax_s, RISCV_SINGLE_PRECISION, rd, rs1, rs2);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FSQRT_S:
        gen_fp_helper_fpr_1fpr_1tcg(gen_helper_fsqrt_s, RISCV_SINGLE_PRECISION, rd, rs1, rm_reg);
        break;
    case OPC_RISC_FEQ_S:
        /* also handles: OPC_RISC_FLT_S, OPC_RISC_FLE_S */
        if (rm == 0x0) {
            gen_fp_helper_gpr_2fpr(gen_helper_fle_s, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rs2);
        } else if (rm == 0x1) {
            gen_fp_helper_gpr_2fpr(gen_helper_flt_s, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rs2);
        } else if (rm == 0x2) {
            gen_fp_helper_gpr_2fpr(gen_helper_feq_s, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rs2);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FCVT_W_S:
        /* also OPC_RISC_FCVT_WU_S, OPC_RISC_FCVT_L_S, OPC_RISC_FCVT_LU_S */
        if (rs2 == 0x0) {        /* FCVT_W_S */
            gen_fp_helper_gpr_1fpr_1tcg(gen_helper_fcvt_w_s, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rm_reg);
        } else if (rs2 == 0x1) { /* FCVT_WU_S */
            gen_fp_helper_gpr_1fpr_1tcg(gen_helper_fcvt_wu_s, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rm_reg);
        } else if (rs2 == 0x2) { /* FCVT_L_S */
#if defined(TARGET_RISCV64)
            gen_fp_helper_gpr_1fpr_1tcg(gen_helper_fcvt_l_s, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else if (rs2 == 0x3) { /* FCVT_LU_S */
#if defined(TARGET_RISCV64)
            gen_fp_helper_gpr_1fpr_1tcg(gen_helper_fcvt_lu_s, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FCVT_S_W:
        /* also OPC_RISC_FCVT_S_WU, OPC_RISC_FCVT_S_L, OPC_RISC_FCVT_S_LU */
        if (rs2 == 0) {          /* FCVT_S_W */
            gen_fp_helper_fpr_1gpr_1tcg(gen_helper_fcvt_s_w, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rm_reg);
        } else if (rs2 == 0x1) { /* FCVT_S_WU */
            gen_fp_helper_fpr_1gpr_1tcg(gen_helper_fcvt_s_wu, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rm_reg);
        } else if (rs2 == 0x2) { /* FCVT_S_L */
#if defined(TARGET_RISCV64)
            gen_fp_helper_fpr_1gpr_1tcg(gen_helper_fcvt_s_l, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else if (rs2 == 0x3) { /* FCVT_S_LU */
#if defined(TARGET_RISCV64)
            gen_fp_helper_fpr_1gpr_1tcg(gen_helper_fcvt_s_lu, RISCV_SINGLE_PRECISION, write_int_rd, rd, rs1, rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FMV_X_S: {
        int fp_ok = gen_new_label();
        int done = gen_new_label();

        // check MSTATUS.FS
        tcg_gen_ld_tl(write_int_rd, cpu_env, offsetof(CPUState, mstatus));
        tcg_gen_andi_tl(write_int_rd, write_int_rd, MSTATUS_FS);
        tcg_gen_brcondi_tl(TCG_COND_NE, write_int_rd, 0x0, fp_ok);
        // MSTATUS_FS field was zero:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        tcg_gen_br(done);

        // proceed with operation
        gen_set_label(fp_ok);
        /* also OPC_RISC_FCLASS_S */
        if (rm == 0x0) {     /* FMV */
#if defined(TARGET_RISCV64)
            tcg_gen_ext32s_tl(write_int_rd, cpu_fpr[rs1]);
#else
            tcg_gen_trunc_i64_i32(write_int_rd, cpu_fpr[rs1]);
#endif
        } else if (rm == 0x1) {
            gen_helper_fclass_s(write_int_rd, cpu_env, cpu_fpr[rs1]);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        gen_set_label(done);
        break;
    }
    case OPC_RISC_FMV_S_X:
    {
        int fp_ok = gen_new_label();
        int done = gen_new_label();

        // check MSTATUS.FS
        tcg_gen_ld_tl(write_int_rd, cpu_env, offsetof(CPUState, mstatus));
        tcg_gen_andi_tl(write_int_rd, write_int_rd, MSTATUS_FS);
        tcg_gen_brcondi_tl(TCG_COND_NE, write_int_rd, 0x0, fp_ok);
        // MSTATUS_FS field was zero:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        tcg_gen_br(done);

        // proceed with operation
        gen_set_label(fp_ok);
        gen_get_gpr(write_int_rd, rs1);
#if defined(TARGET_RISCV64)
        tcg_gen_mov_tl(cpu_fpr[rd], write_int_rd);
#else
        tcg_gen_extu_i32_i64(cpu_fpr[rd], write_int_rd);
#endif
        gen_box_float(RISCV_SINGLE_PRECISION, cpu_fpr[rd]);
        gen_set_label(done);
        break;
    }
    /* double */
    case OPC_RISC_FADD_D:
        gen_helper_fadd_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2], rm_reg);
        break;
    case OPC_RISC_FSUB_D:
        gen_helper_fsub_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2], rm_reg);
        break;
    case OPC_RISC_FMUL_D:
        gen_helper_fmul_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2], rm_reg);
        break;
    case OPC_RISC_FDIV_D:
        gen_helper_fdiv_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2], rm_reg);
        break;
    case OPC_RISC_FSGNJ_D:
        gen_fsgnj(dc, rd, rs1, rs2, rm, RISCV_DOUBLE_PRECISION);
        break;
    case OPC_RISC_FMIN_D:
        /* also OPC_RISC_FMAX_D */
        if (rm == 0x0) {
            gen_helper_fmin_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else if (rm == 0x1) {
            gen_helper_fmax_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FCVT_S_D: {
        TCGv_i64 rs1_boxed = tcg_temp_local_new_i64();
        if (rs2 == 0x1) {
            gen_helper_fcvt_s_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], rm_reg);
            gen_box_float(RISCV_SINGLE_PRECISION, cpu_fpr[rd]);
        } else if (rs2 == 0x2) {
            gen_unbox_float(RISCV_HALF_PRECISION, env, rs1_boxed, cpu_fpr[rs1]);
            gen_helper_fcvt_s_h(cpu_fpr[rd], cpu_env, rs1_boxed, rm_reg);
            gen_box_float(RISCV_SINGLE_PRECISION, cpu_fpr[rd]);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        tcg_temp_free_i64(rs1_boxed);
        break;
    }
    case OPC_RISC_FCVT_D_S: {
        TCGv_i64 rs1_boxed = tcg_temp_local_new_i64();
        if (rs2 == 0x0) {
            gen_unbox_float(RISCV_SINGLE_PRECISION, env, rs1_boxed, cpu_fpr[rs1]);
            gen_helper_fcvt_d_s(cpu_fpr[rd], cpu_env, rs1_boxed, rm_reg);
        } else if (rs2 == 0x2) {
            gen_unbox_float(RISCV_HALF_PRECISION, env, rs1_boxed, cpu_fpr[rs1]);
            gen_helper_fcvt_d_h(cpu_fpr[rd], cpu_env, rs1_boxed, rm_reg);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        tcg_temp_free_i64(rs1_boxed);
        break;
    }
    case OPC_RISC_FSQRT_D:
        gen_helper_fsqrt_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], rm_reg);
        break;
    case OPC_RISC_FEQ_D:
        /* also OPC_RISC_FLT_D, OPC_RISC_FLE_D */
        if (rm == 0x0) {
            gen_helper_fle_d(write_int_rd, cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else if (rm == 0x1) {
            gen_helper_flt_d(write_int_rd, cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else if (rm == 0x2) {
            gen_helper_feq_d(write_int_rd, cpu_env, cpu_fpr[rs1], cpu_fpr[rs2]);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        break;
    case OPC_RISC_FCVT_W_D:
        /* also OPC_RISC_FCVT_WU_D, OPC_RISC_FCVT_L_D, OPC_RISC_FCVT_LU_D */
        if (rs2 == 0x0) {
            gen_helper_fcvt_w_d(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
        } else if (rs2 == 0x1) {
            gen_helper_fcvt_wu_d(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
        } else if (rs2 == 0x2) {
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_l_d(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else if (rs2 == 0x3) {
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_lu_d(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        break;
    case OPC_RISC_FCVT_D_W:
        /* also OPC_RISC_FCVT_D_WU, OPC_RISC_FCVT_D_L, OPC_RISC_FCVT_D_LU */
        gen_get_gpr(write_int_rd, rs1);
        if (rs2 == 0x0) {
            gen_helper_fcvt_d_w(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
        } else if (rs2 == 0x1) {
            gen_helper_fcvt_d_wu(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
        } else if (rs2 == 0x2) {
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_d_l(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else if (rs2 == 0x3) {
#if defined(TARGET_RISCV64)
            gen_helper_fcvt_d_lu(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
#if defined(TARGET_RISCV64)
    case OPC_RISC_FMV_X_D:
    {
        int fp_ok = gen_new_label();
        int done = gen_new_label();

        // check MSTATUS.FS
        tcg_gen_ld_tl(write_int_rd, cpu_env, offsetof(CPUState, mstatus));
        tcg_gen_andi_tl(write_int_rd, write_int_rd, MSTATUS_FS);
        tcg_gen_brcondi_tl(TCG_COND_NE, write_int_rd, 0x0, fp_ok);
        // MSTATUS_FS field was zero:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        tcg_gen_br(done);

        // proceed with operation
        gen_set_label(fp_ok);
        /* also OPC_RISC_FCLASS_D */
        if (rm == 0x0) {     /* FMV */
            tcg_gen_mov_tl(write_int_rd, cpu_fpr[rs1]);
        } else if (rm == 0x1) {
            gen_helper_fclass_d(write_int_rd, cpu_env, cpu_fpr[rs1]);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        gen_set_label(done);
        break;
    }
    case OPC_RISC_FMV_D_X:
    {
        int fp_ok = gen_new_label();
        int done = gen_new_label();

        // check MSTATUS.FS
        tcg_gen_ld_tl(write_int_rd, cpu_env, offsetof(CPUState, mstatus));
        tcg_gen_andi_tl(write_int_rd, write_int_rd, MSTATUS_FS);
        tcg_gen_brcondi_tl(TCG_COND_NE, write_int_rd, 0x0, fp_ok);
        // MSTATUS_FS field was zero:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        tcg_gen_br(done);

        // proceed with operation
        gen_set_label(fp_ok);
        gen_get_gpr(write_int_rd, rs1);
        tcg_gen_mov_tl(cpu_fpr[rd], write_int_rd);
        gen_set_label(done);
        break;
    }
#endif
    /* half-precision */
    case OPC_RISC_FADD_H:
        gen_fp_helper_fpr_2fpr_1tcg(gen_helper_fadd_h, RISCV_HALF_PRECISION, rd, rs1, rs2, rm_reg);
        break;
    case OPC_RISC_FSUB_H:
        gen_fp_helper_fpr_2fpr_1tcg(gen_helper_fsub_h, RISCV_HALF_PRECISION, rd, rs1, rs2, rm_reg);
        break;
    case OPC_RISC_FMUL_H:
        gen_fp_helper_fpr_2fpr_1tcg(gen_helper_fmul_h, RISCV_HALF_PRECISION, rd, rs1, rs2, rm_reg);
        break;
    case OPC_RISC_FDIV_H:
        gen_fp_helper_fpr_2fpr_1tcg(gen_helper_fdiv_h, RISCV_HALF_PRECISION, rd, rs1, rs2, rm_reg);
        break;
    case OPC_RISC_FSGNJ_H:
        gen_fsgnj(dc, rd, rs1, rs2, rm, RISCV_HALF_PRECISION);
        break;
    case OPC_RISC_FMIN_H:
        /* also OPC_RISC_FMAX_H */
        if (rm == 0x0) {
            gen_fp_helper_fpr_2fpr(gen_helper_fmin_h, RISCV_HALF_PRECISION, rd, rs1, rs2);
        } else if (rm == 0x1) {
            gen_fp_helper_fpr_2fpr(gen_helper_fmax_h, RISCV_HALF_PRECISION, rd, rs1, rs2);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FCVT_H_S:
        {
            TCGv_i64 rs1_boxed = tcg_temp_local_new_i64();
            if (rs2 == 0x0) {
                gen_unbox_float(RISCV_SINGLE_PRECISION, env, rs1_boxed, cpu_fpr[rs1]); 
                gen_helper_fcvt_h_s(cpu_fpr[rd], cpu_env, rs1_boxed, rm_reg);
                gen_box_float(RISCV_HALF_PRECISION, cpu_fpr[rd]);
            } else if (rs2 == 0x1) {
                gen_helper_fcvt_h_d(cpu_fpr[rd], cpu_env, cpu_fpr[rs1], rm_reg);
                gen_box_float(RISCV_HALF_PRECISION, cpu_fpr[rd]);
            } else {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            }
            tcg_temp_free_i64(rs1_boxed);
            break;
        }
    case OPC_RISC_FSQRT_H:
        gen_fp_helper_fpr_1fpr_1tcg(gen_helper_fsqrt_h, RISCV_HALF_PRECISION, rd, rs1, rm_reg);
        break;
    case OPC_RISC_FEQ_H:
        /* also OPC_RISC_FLT_H, OPC_RISC_FLE_H */
        if (rm == 0x0) {
            gen_fp_helper_gpr_2fpr(gen_helper_fle_h, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rs2);
        } else if (rm == 0x1) {
            gen_fp_helper_gpr_2fpr(gen_helper_flt_h, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rs2);
        } else if (rm == 0x2) {
            gen_fp_helper_gpr_2fpr(gen_helper_feq_h, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rs2);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FCVT_W_H:
        /* also OPC_RISC_FCVT_WU_H, OPC_RISC_FCVT_L_H, OPC_RISC_FCVT_LU_H */
        if (rs2 == 0x0) {
            gen_fp_helper_gpr_1fpr_1tcg(gen_helper_fcvt_w_h, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rm_reg);
        } else if (rs2 == 0x1) {
            gen_fp_helper_gpr_1fpr_1tcg(gen_helper_fcvt_wu_h, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rm_reg);
        } else if (rs2 == 0x2) {
#if defined(TARGET_RISCV64)
            gen_fp_helper_gpr_1fpr_1tcg(gen_helper_fcvt_l_h, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else if (rs2 == 0x3) {
#if defined(TARGET_RISCV64)
            gen_fp_helper_gpr_1fpr_1tcg(gen_helper_fcvt_lu_h, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FCVT_H_W:
        /* also OPC_RISC_FCVT_H_WU, OPC_RISC_FCVT_H_L, OPC_RISC_FCVT_H_LU */
        if (rs2 == 0x0) {
            gen_fp_helper_fpr_1gpr_1tcg(gen_helper_fcvt_h_w, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rm_reg);
        } else if (rs2 == 0x1) {
            gen_fp_helper_fpr_1gpr_1tcg(gen_helper_fcvt_h_wu, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rm_reg);
        } else if (rs2 == 0x2) {
#if defined(TARGET_RISCV64)
            gen_fp_helper_fpr_1gpr_1tcg(gen_helper_fcvt_h_l, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else if (rs2 == 0x3) {
#if defined(TARGET_RISCV64)
            gen_fp_helper_fpr_1gpr_1tcg(gen_helper_fcvt_h_lu, RISCV_HALF_PRECISION, write_int_rd, rd, rs1, rm_reg);
#else
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
#endif
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case OPC_RISC_FMV_X_H:
        /* also OPC_RISC_FCLASS_H */
        if (rm == 0x0) {     /* FMV */
            gen_helper_fmv_x_h(write_int_rd, cpu_env, cpu_fpr[rs1], rm_reg);
        } else if (rm == 0x1) {
            gen_helper_fclass_h(write_int_rd, cpu_env, cpu_fpr[rs1]);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        gen_set_gpr(rd, write_int_rd);
        break;
    case OPC_RISC_FMV_H_X:
        gen_get_gpr(write_int_rd, rs1);
        gen_helper_fmv_h_x(cpu_fpr[rd], cpu_env, write_int_rd, rm_reg);
        gen_box_float(RISCV_HALF_PRECISION, cpu_fpr[rd]);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free_i64(rm_reg);
    tcg_temp_free(write_int_rd);

#ifdef DL_TRACE_ARITH
    // DataLab: helper call
    gen_sync_pc(dc);
    TCGv_i32 topc = tcg_const_i32(opc);
    TCGv_i32 topclass = tcg_const_i32(DL_RISC_FP_ARITH);
    gen_helper_log_inst_arith(cpu_env, topc, topclass);

    // DataLab: free temporary variables
    tcg_temp_free_i32(topc);
    tcg_temp_free_i32(topclass);
#endif
}

static void gen_system(DisasContext *dc, uint32_t opc, int rd, int rs1, int rs2, int funct12)
{
    gen_sync_pc(dc);
    if (opc == OPC_RISC_ECALL) {
        // This group uses both `I-type` and `R-type` instruction formats
        // It's easier to start narrowing with the shorter function code
        int funct7 = funct12 >> 5;

        switch (funct7) {
        case 0x0:
            switch(rs2) {
            case 0x0:  /* ECALL */
                /* always generates U-level ECALL, fixed in do_interrupt handler */
                generate_exception(dc, RISCV_EXCP_U_ECALL);
                gen_exit_tb_no_chaining(dc->base.tb);
                dc->base.is_jmp = DISAS_BRANCH;
                break;
            case 0x1: /* EBREAK */
                generate_exception(dc, RISCV_EXCP_BREAKPOINT);
                gen_exit_tb_no_chaining(dc->base.tb);
                dc->base.is_jmp = DISAS_BRANCH;
                break;
            case 0x2: /* URET */
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            default:
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            }
            break;
        case 0x8:
            switch(rs2) {
            case 0x2: /* SRET */
                gen_helper_sret(cpu_pc, cpu_env, cpu_pc);
                gen_exit_tb_no_chaining(dc->base.tb);
                dc->base.is_jmp = DISAS_BRANCH;
                break;
            case 0x4: /* SFENCE.VM */
                gen_helper_tlb_flush(cpu_env);
                break;
            case 0x5: /* WFI */
                tcg_gen_movi_tl(cpu_pc, dc->npc);
                gen_helper_wfi(cpu_env);
                gen_exit_tb_no_chaining(dc->base.tb);
                dc->base.is_jmp = DISAS_BRANCH;
                break;
            default:
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            }
            break;
        case 0x9: /* SFENCE.VMA */
            /* TODO: handle ASID specific fences */
            gen_helper_tlb_flush(cpu_env);
            break;
        case 0x10: /* HRET */
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        case 0x18: /* MRET */
            gen_helper_mret(cpu_pc, cpu_env, cpu_pc);
            gen_exit_tb_no_chaining(dc->base.tb);
            dc->base.is_jmp = DISAS_BRANCH;
            break;
        case 0x3d: /* DRET */
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        default:
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
    }
    else
    {
        if (!riscv_has_additional_ext(cpu, RISCV_FEATURE_ZICSR)) {
            int instruction_length = decode_instruction_length(dc->opcode);
            tlib_printf(LOG_LEVEL_ERROR, "RISC-V Zicsr instruction set is not enabled for this CPU! In future release this configuration will lead to an illegal instruction exception. PC: 0x%llx, opcode: 0x%0*llx",
                dc->base.pc, /* padding */ 2 * instruction_length, format_opcode(dc->opcode, instruction_length));
        }
        TCGv source1, csr_store, dest, rs1_pass, imm_rs1;
        source1 = tcg_temp_new();
        csr_store = tcg_temp_new();
        dest = tcg_temp_new();
        rs1_pass = tcg_temp_new();
        imm_rs1 = tcg_temp_new();
        gen_get_gpr(source1, rs1);
        tcg_gen_movi_tl(rs1_pass, rs1);
        tcg_gen_movi_tl(csr_store, funct12); /* copy into temp reg to feed to helper */
        tcg_gen_movi_tl(imm_rs1, rs1);

        switch (opc) {
        case OPC_RISC_CSRRW:
            gen_helper_csrrw(dest, cpu_env, source1, csr_store);
            break;
        case OPC_RISC_CSRRS:
            gen_helper_csrrs(dest, cpu_env, source1, csr_store, rs1_pass);
            break;
        case OPC_RISC_CSRRC:
            gen_helper_csrrc(dest, cpu_env, source1, csr_store, rs1_pass);
            break;
        case OPC_RISC_CSRRWI:
            gen_helper_csrrw(dest, cpu_env, imm_rs1, csr_store);
            break;
        case OPC_RISC_CSRRSI:
            gen_helper_csrrs(dest, cpu_env, imm_rs1, csr_store, rs1_pass);
            break;
        case OPC_RISC_CSRRCI:
            gen_helper_csrrc(dest, cpu_env, imm_rs1, csr_store, rs1_pass);
            break;
        default:
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }

        gen_set_gpr(rd, dest);
        /* end tb since we may be changing priv modes, to get mmu_index right */
        tcg_gen_movi_tl(cpu_pc, dc->npc);
        gen_exit_tb_no_chaining(dc->base.tb);
        dc->base.is_jmp = DISAS_BRANCH;

        tcg_temp_free(source1);
        tcg_temp_free(csr_store);
        tcg_temp_free(dest);
        tcg_temp_free(rs1_pass);
        tcg_temp_free(imm_rs1);
    }
}

// Vector helpers require 128-bit ints which aren't supported on 32-bit hosts.
#if HOST_LONG_BITS != 32
static void gen_v_cfg(DisasContext *dc, uint32_t opc, int rd, int rs1, int rs2, int imm)
{
    TCGv rs1_value, rs2_value, zimm, returned_vl, rd_index, rs1_index, rs1_is_uimm;

    zimm = tcg_temp_new();
    rd_index = tcg_temp_new();
    rs1_index = tcg_temp_new();
    rs1_value = tcg_temp_new();
    rs2_value = tcg_temp_new();

    if(opc == OPC_RISC_VSETIVLI) {
        // in vsetivli the imm field is [9:0] rather than [11:0]
        imm = imm & ((1 << 10) - 1);
    }
    else if(opc == OPC_RISC_VSETVLI_0 || opc == OPC_RISC_VSETVLI_1) {
        // in vsetvli the imm field is [10:0] rather than [11:0]
        imm = imm & ((1 << 11) - 1);
    }


    tcg_gen_movi_tl(rd_index, rd);
    tcg_gen_movi_tl(rs1_index, rs1);
    tcg_gen_movi_tl(zimm, imm);
    gen_get_gpr(rs1_value, rs1);
    gen_get_gpr(rs2_value, rs2);

    rs1_is_uimm = tcg_temp_new();
    if (opc == OPC_RISC_VSETIVLI) {
        tcg_gen_movi_i32(rs1_is_uimm, 1);
    } else {
        tcg_gen_movi_i32(rs1_is_uimm, 0);
    }

    gen_sync_pc(dc);
    returned_vl = tcg_temp_new();

    switch (opc) {
        case OPC_RISC_VSETVL:
            gen_helper_vsetvl(returned_vl, cpu_env, rd_index, rs1_index, rs1_value, rs2_value,
                              rs1_is_uimm);
            break;
        case OPC_RISC_VSETVLI_0:
        case OPC_RISC_VSETVLI_1:
            gen_helper_vsetvl(returned_vl, cpu_env, rd_index, rs1_index, rs1_value, zimm,
                              rs1_is_uimm);
            break;
        case OPC_RISC_VSETIVLI:
            gen_helper_vsetvl(returned_vl, cpu_env, rd_index, rs1_index, rs1_index, zimm,
                              rs1_is_uimm);
            break;
        default:
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
    }
    gen_set_gpr(rd, returned_vl);

    tcg_temp_free(rs1_value);
    tcg_temp_free(rs2_value);
    tcg_temp_free(returned_vl);
    tcg_temp_free(rs1_index);
    tcg_temp_free(rs1_is_uimm);
}

static void gen_v_opivv(DisasContext *dc, uint8_t funct6, int vd, int vs1, int vs2, uint8_t vm)
{
    generate_vill_check(dc);
    TCGv_i32 t_vd, t_vs1, t_vs2;
    t_vd = tcg_temp_new_i32();
    t_vs1 = tcg_temp_new_i32();
    t_vs2 = tcg_temp_new_i32();
    tcg_gen_movi_i32(t_vd, vd);
    tcg_gen_movi_i32(t_vs1, vs1);
    tcg_gen_movi_i32(t_vs2, vs2);

    switch (funct6) {
    case RISC_V_FUNCT_ADD:
        if (vm) {
            gen_helper_vadd_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vadd_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SUB:
        if (vm) {
            gen_helper_vsub_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vsub_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MINU:
        if (vm) {
            gen_helper_vminu_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vminu_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MIN:
        if (vm) {
            gen_helper_vmin_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmin_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MAXU:
        if (vm) {
            gen_helper_vmaxu_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmaxu_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MAX:
        if (vm) {
            gen_helper_vmax_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmax_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_AND:
        if (vm) {
            gen_helper_vand_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vand_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_OR:
        if (vm) {
            gen_helper_vor_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vor_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_XOR:
        if (vm) {
            gen_helper_vxor_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vxor_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_RGATHER:
        if (vm) {
            gen_helper_vrgather_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vrgather_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_RGATHEREI16:
        if (vm) {
            gen_helper_vrgatherei16_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vrgatherei16_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_ADC:
        if (vm) {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        } else {
            if (!vd) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            }
            gen_helper_vadc_vvm(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MADC:
        if (vm) {
            gen_helper_vmadc_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmadc_vvm(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SBC:
        if (vm) {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        } else {
            if (!vd) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            }
            gen_helper_vsbc_vvm(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MSBC:
        if (vm) {
            gen_helper_vmsbc_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmsbc_vvm(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MERGE_MV:
        if (vm) {
            if (vs2) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            } else {
                gen_helper_vmv_ivv(cpu_env, t_vd, t_vs1);
            }
        } else {
            gen_helper_vmerge_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MSEQ:
        if (vm) {
            gen_helper_vmseq_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmseq_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MSNE:
        if (vm) {
            gen_helper_vmsne_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmsne_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MSLTU:
        if (vm) {
            gen_helper_vmsltu_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmsltu_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MSLT:
        if (vm) {
            gen_helper_vmslt_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmslt_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MSLEU:
        if (vm) {
            gen_helper_vmsleu_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmsleu_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MSLE:
        if (vm) {
            gen_helper_vmsle_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmsle_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SADDU:
        if (vm) {
            gen_helper_vsaddu_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vsaddu_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SADD:
        if (vm) {
            gen_helper_vsadd_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vsadd_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SSUBU:
        if (vm) {
            gen_helper_vssubu_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vssubu_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SSUB:
        if (vm) {
            gen_helper_vssub_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vssub_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SLL:
        if (vm) {
            gen_helper_vsll_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vsll_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SMUL:
        if (vm) {
            gen_helper_vsmul_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vsmul_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SRL:
        if (vm) {
            gen_helper_vsrl_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vsrl_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SRA:
        if (vm) {
            gen_helper_vsra_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vsra_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SSRL:
        if (vm) {
            gen_helper_vssrl_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vssrl_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_SSRA:
        if (vm) {
            gen_helper_vssra_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vssra_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_NSRL:
        if (vm) {
            gen_helper_vnsrl_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vnsrl_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_NSRA:
        if (vm) {
            gen_helper_vnsra_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vnsra_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_NCLIPU:
        if (vm) {
            gen_helper_vnclipu_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vnclipu_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_NCLIP:
        if (vm) {
            gen_helper_vnclip_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vnclip_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WREDSUMU:
        if (vm) {
            gen_helper_vwredsumu_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwredsumu_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WREDSUM:
        if (vm) {
            gen_helper_vwredsum_ivv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwredsum_ivv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free_i32(t_vd);
    tcg_temp_free_i32(t_vs1);
    tcg_temp_free_i32(t_vs2);
}

// common or mutually exclusive operations for vi and vx
static void gen_v_opivt(DisasContext *dc, uint8_t funct6, int vd, int vs2, TCGv t, uint8_t vm)
{
    TCGv_i32 t_vd, t_vs2;
    t_vd = tcg_temp_new_i32();
    t_vs2 = tcg_temp_new_i32();
    tcg_gen_movi_i32(t_vd, vd);
    tcg_gen_movi_i32(t_vs2, vs2);

    switch (funct6) {
    // Common for vi and vx
    case RISC_V_FUNCT_ADD:
        if (vm) {
            gen_helper_vadd_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vadd_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_RSUB:
        if (vm) {
            gen_helper_vrsub_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vrsub_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_AND:
        if (vm) {
            gen_helper_vand_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vand_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_OR:
        if (vm) {
            gen_helper_vor_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vor_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_XOR:
        if (vm) {
            gen_helper_vxor_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vxor_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_RGATHER:
        if (vm) {
            gen_helper_vrgather_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vrgather_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SLIDEUP:
        if (vm) {
            gen_helper_vslideup_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vslideup_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SLIDEDOWN:
        if (vm) {
            gen_helper_vslidedown_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vslidedown_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_ADC:
        if (vm) {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        } else {
            if (!vd) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            }
            gen_helper_vadc_vi(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MADC:
        if (vm) {
            gen_helper_vmadc_vi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmadc_vim(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MERGE_MV:
        if (vm) {
            if (vs2) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            } else {
                gen_helper_vmv_ivi(cpu_env, t_vd, t);
            }
        } else {
            gen_helper_vmerge_ivi(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MSEQ:
        if (vm) {
            gen_helper_vmseq_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmseq_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MSNE:
        if (vm) {
            gen_helper_vmsne_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmsne_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MSLEU:
        if (vm) {
            gen_helper_vmsleu_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmsleu_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MSLE:
        if (vm) {
            gen_helper_vmsle_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmsle_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MSGTU:
        if (vm) {
            gen_helper_vmsgtu_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmsgtu_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MSGT:
        if (vm) {
            gen_helper_vmsgt_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmsgt_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SADDU:
        if (vm) {
            gen_helper_vsaddu_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vsaddu_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SADD:
        if (vm) {
            gen_helper_vsadd_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vsadd_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SLL:
        if (vm) {
            gen_helper_vsll_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vsll_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SRL:
        if (vm) {
            gen_helper_vsrl_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vsrl_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SRA:
        if (vm) {
            gen_helper_vsra_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vsra_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SSRL:
        if (vm) {
            gen_helper_vssrl_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vssrl_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SSRA:
        if (vm) {
            gen_helper_vssra_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vssra_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_NSRL:
        if (vm) {
            gen_helper_vnsrl_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vnsrl_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_NSRA:
        if (vm) {
            gen_helper_vnsra_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vnsra_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_NCLIPU:
        if (vm) {
            gen_helper_vnclipu_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vnclipu_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_NCLIP:
        if (vm) {
            gen_helper_vnclip_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vnclip_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    // defined for vi and reserved for vx
    // reserved for vi and defined for vx
    case RISC_V_FUNCT_SUB:
        tcg_gen_neg_i64(t, t);
        if (vm) {
            gen_helper_vadd_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vadd_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MINU:
        if (vm) {
            gen_helper_vminu_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vminu_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MIN:
        if (vm) {
            gen_helper_vmin_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmin_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MAXU:
        if (vm) {
            gen_helper_vmaxu_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmaxu_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MAX:
        if (vm) {
            gen_helper_vmax_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmax_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SBC:
        if (vm) {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        } else {
            if (!vd) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                break;
            }
            gen_helper_vsbc_vi(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MSBC:
        if (vm) {
            gen_helper_vmsbc_vi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmsbc_vim(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MSLTU:
        if (vm) {
            gen_helper_vmsltu_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmsltu_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_MSLT:
        if (vm) {
            gen_helper_vmslt_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vmslt_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SSUBU:
        if (vm) {
            gen_helper_vssubu_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vssubu_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    case RISC_V_FUNCT_SSUB:
        if (vm) {
            gen_helper_vssub_ivi(cpu_env, t_vd, t_vs2, t);
        } else {
            gen_helper_vssub_ivi_m(cpu_env, t_vd, t_vs2, t);
        }
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free_i32(t_vd);
    tcg_temp_free_i32(t_vs2);
}

static void gen_v_opivi(DisasContext *dc, uint8_t funct6, int vd, int rs1, int vs2, uint8_t vm)
{
    if (funct6 != RISC_V_FUNCT_MV_NF_R) {
        generate_vill_check(dc);
    }
    int64_t simm5 = rs1;
    TCGv t_simm5;
    TCGv_i32 t_vd, t_vs2;
    t_simm5 = tcg_temp_new();

    switch (funct6) {
    // Common for vi and vx
    // zero-extended immediate
    case RISC_V_FUNCT_NSRL:
    case RISC_V_FUNCT_NSRA:
    case RISC_V_FUNCT_NCLIPU:
    case RISC_V_FUNCT_NCLIP:
    case RISC_V_FUNCT_SLIDEUP:
    case RISC_V_FUNCT_SLIDEDOWN:
    case RISC_V_FUNCT_RGATHER:
    case RISC_V_FUNCT_SLL:
    case RISC_V_FUNCT_SRL:
    case RISC_V_FUNCT_SRA:
    case RISC_V_FUNCT_SSRL:
    case RISC_V_FUNCT_SSRA:
        tcg_gen_movi_tl(t_simm5, simm5);
        gen_v_opivt(dc, funct6, vd, vs2, t_simm5, vm);
        break;
    // sign-extended immediate
    case RISC_V_FUNCT_ADD:
    case RISC_V_FUNCT_RSUB:
    case RISC_V_FUNCT_AND:
    case RISC_V_FUNCT_OR:
    case RISC_V_FUNCT_XOR:
    case RISC_V_FUNCT_ADC:
    case RISC_V_FUNCT_MADC:
    case RISC_V_FUNCT_MERGE_MV:
    case RISC_V_FUNCT_MSEQ:
    case RISC_V_FUNCT_MSNE:
    case RISC_V_FUNCT_MSLEU:
    case RISC_V_FUNCT_MSLE:
    case RISC_V_FUNCT_MSGTU:
    case RISC_V_FUNCT_MSGT:
    case RISC_V_FUNCT_SADDU:
    case RISC_V_FUNCT_SADD:
    // Reserved for vx
        simm5 = rs1 >= 0x10 ? (0xffffffffffffffe0) | rs1 : rs1;
        tcg_gen_movi_tl(t_simm5, simm5);
        gen_v_opivt(dc, funct6, vd, vs2, t_simm5, vm);
        break;
    // Conflicting
    case RISC_V_FUNCT_MV_NF_R:
        t_vd = tcg_temp_new_i32();
        t_vs2 = tcg_temp_new_i32();
        tcg_gen_movi_i32(t_vd, vd);
        tcg_gen_movi_i32(t_vs2, vs2);

        switch (rs1) {
        case 0:
            gen_helper_vmv1r_v(cpu_env, t_vd, t_vs2);
            break;
        case 1:
            gen_helper_vmv2r_v(cpu_env, t_vd, t_vs2);
            break;
        case 3:
            gen_helper_vmv4r_v(cpu_env, t_vd, t_vs2);
            break;
        case 7:
            gen_helper_vmv8r_v(cpu_env, t_vd, t_vs2);
            break;
        default:
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        tcg_temp_free_i32(t_vd);
        tcg_temp_free_i32(t_vs2);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free(t_simm5);
}

static void gen_v_opivx(DisasContext *dc, uint8_t funct6, int vd, int rs1, int vs2, uint8_t vm)
{
    generate_vill_check(dc);
    TCGv_i32 t_vd, t_vs2;
    TCGv t_tl;
    t_tl = tcg_temp_new();
    gen_get_gpr(t_tl, rs1);

    switch (funct6) {
    // Common for vi and vx
    case RISC_V_FUNCT_ADD:
    case RISC_V_FUNCT_RSUB:
    case RISC_V_FUNCT_AND:
    case RISC_V_FUNCT_OR:
    case RISC_V_FUNCT_XOR:
    case RISC_V_FUNCT_RGATHER:
    case RISC_V_FUNCT_SLIDEUP:
    case RISC_V_FUNCT_SLIDEDOWN:
    case RISC_V_FUNCT_ADC:
    case RISC_V_FUNCT_MADC:
    case RISC_V_FUNCT_MERGE_MV:
    case RISC_V_FUNCT_MSEQ:
    case RISC_V_FUNCT_MSNE:
    case RISC_V_FUNCT_MSLEU:
    case RISC_V_FUNCT_MSLE:
    case RISC_V_FUNCT_MSGTU:
    case RISC_V_FUNCT_MSGT:
    case RISC_V_FUNCT_SADDU:
    case RISC_V_FUNCT_SADD:
    case RISC_V_FUNCT_SLL:
    case RISC_V_FUNCT_SRL:
    case RISC_V_FUNCT_SRA:
    case RISC_V_FUNCT_SSRL:
    case RISC_V_FUNCT_SSRA:
    case RISC_V_FUNCT_NSRL:
    case RISC_V_FUNCT_NSRA:
    case RISC_V_FUNCT_NCLIPU:
    case RISC_V_FUNCT_NCLIP:
    // Reserved for vi
    case RISC_V_FUNCT_SUB:
    case RISC_V_FUNCT_MINU:
    case RISC_V_FUNCT_MIN:
    case RISC_V_FUNCT_MAXU:
    case RISC_V_FUNCT_MAX:
    case RISC_V_FUNCT_SBC:
    case RISC_V_FUNCT_MSBC:
    case RISC_V_FUNCT_MSLTU:
    case RISC_V_FUNCT_MSLT:
    case RISC_V_FUNCT_SSUBU:
    case RISC_V_FUNCT_SSUB:
        gen_v_opivt(dc, funct6, vd, vs2, t_tl, vm);
        break;
    // Conflicting
    case RISC_V_FUNCT_SMUL:
        t_vd = tcg_temp_new_i32();
        t_vs2 = tcg_temp_new_i32();
        tcg_gen_movi_i32(t_vd, vd);
        tcg_gen_movi_i32(t_vs2, vs2);
        if (vm) {
            gen_helper_vsmul_ivx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vsmul_ivx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        tcg_temp_free_i32(t_vd);
        tcg_temp_free_i32(t_vs2);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free(t_tl);
}

static void gen_v_opmvv(DisasContext *dc, uint8_t funct6, int vd, int vs1, int vs2, uint8_t vm)
{
    generate_vill_check(dc);
    TCGv_i32 t_vd, t_vs1, t_vs2;
    TCGv t_tl;
    t_vd = tcg_temp_new_i32();
    t_vs1 = tcg_temp_new_i32();
    t_vs2 = tcg_temp_new_i32();
    t_tl = tcg_temp_new();
    tcg_gen_movi_i32(t_vd, vd);
    tcg_gen_movi_i32(t_vs1, vs1);
    tcg_gen_movi_i32(t_vs2, vs2);

    switch (funct6) {
    case RISC_V_FUNCT_REDSUM:
        if (vm) {
            gen_helper_vredsum_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vredsum_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_REDAND:
        if (vm) {
            gen_helper_vredand_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vredand_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_REDOR:
        if (vm) {
            gen_helper_vredor_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vredor_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_REDXOR:
        if (vm) {
            gen_helper_vredxor_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vredxor_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_REDMINU:
        if (vm) {
            gen_helper_vredminu_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vredminu_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_REDMIN:
        if (vm) {
            gen_helper_vredmin_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vredmin_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_REDMAXU:
        if (vm) {
            gen_helper_vredmaxu_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vredmaxu_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_REDMAX:
        if (vm) {
            gen_helper_vredmax_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vredmax_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_AADDU:
        if (vm) {
            gen_helper_vaaddu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vaaddu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_AADD:
        if (vm) {
            gen_helper_vaadd_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vaadd_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_ASUBU:
        if (vm) {
            gen_helper_vasubu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vasubu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_ASUB:
        if (vm) {
            gen_helper_vasub_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vasub_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WXUNARY0:
        switch (vs1) {
        case 0x0:
            if (vm) {
                gen_helper_vmv_xs(t_tl, cpu_env, t_vs2);
                gen_set_gpr(vd, t_tl);
            } else {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            }
            break;
        case 0x10:
            if (vm) {
                gen_helper_vpopc(t_tl, cpu_env, t_vs2);
            } else {
                gen_helper_vpopc_m(t_tl, cpu_env, t_vs2);
            }
            gen_set_gpr(vd, t_tl);
            break;
        case 0x11:
            if (vm) {
                gen_helper_vfirst(t_tl, cpu_env, t_vs2);
            } else {
                gen_helper_vfirst_m(t_tl, cpu_env, t_vs2);
            }
            gen_set_gpr(vd, t_tl);
            break;
        default:
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        break;
    case RISC_V_FUNCT_XUNARY0:
        switch (vs1) {
        case 2:
            if (vm) {
                gen_helper_vzext_vf8(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vzext_vf8_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 3:
            if (vm) {
                gen_helper_vsext_vf8(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vsext_vf8_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 4:
            if (vm) {
                gen_helper_vzext_vf4(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vzext_vf4_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 5:
            if (vm) {
                gen_helper_vsext_vf4(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vsext_vf4_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 6:
            if (vm) {
                gen_helper_vzext_vf2(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vzext_vf2_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 7:
            if (vm) {
                gen_helper_vsext_vf2(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vsext_vf2_m(cpu_env, t_vd, t_vs2);
            }
            break;
        default:
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        break;
    case RISC_V_FUNCT_MUNARY0:
        switch (vs1) {
        case 0x1:
            if (vm) {
                gen_helper_vmsbf(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vmsbf_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x2:
            if (vm) {
                gen_helper_vmsof(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vmsof_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x3:
            if (vm) {
                gen_helper_vmsif(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vmsif_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x10:
            if (vm) {
                gen_helper_viota(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_viota_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x11:
            if (vs2) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            } else if (vm) {
                gen_helper_vid(cpu_env, t_vd);
            } else {
                gen_helper_vid_m(cpu_env, t_vd);
            }
            break;
        default:
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        break;
    case RISC_V_FUNCT_COMPRESS:
        if (vm) {
            gen_helper_vcompress_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_MANDNOT:
        if (vm) {
            gen_helper_vmandnot_mm(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_MAND:
        if (vm) {
            gen_helper_vmand_mm(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_MOR:
        if (vm) {
            gen_helper_vmor_mm(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_MXOR:
        if (vm) {
            gen_helper_vmxor_mm(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_MORNOT:
        if (vm) {
            gen_helper_vmornot_mm(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_MNAND:
        if (vm) {
            gen_helper_vmnand_mm(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_MNOR:
        if (vm) {
            gen_helper_vmnor_mm(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_MXNOR:
        if (vm) {
            gen_helper_vmxnor_mm(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_DIVU:
        if (vm) {
            gen_helper_vdivu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vdivu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_DIV:
        if (vm) {
            gen_helper_vdiv_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vdiv_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_REMU:
        if (vm) {
            gen_helper_vremu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vremu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_REM:
        if (vm) {
            gen_helper_vrem_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vrem_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MULHU:
        if (vm) {
            gen_helper_vmulhu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmulhu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MUL:
        if (vm) {
            gen_helper_vmul_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmul_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MULHSU:
        if (vm) {
            gen_helper_vmulhsu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmulhsu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MULH:
        if (vm) {
            gen_helper_vmulh_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmulh_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MADD:
        if (vm) {
            gen_helper_vmadd_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmadd_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_NMSUB:
        if (vm) {
            gen_helper_vnmsub_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vnmsub_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MACC:
        if (vm) {
            gen_helper_vmacc_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vmacc_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_NMSAC:
        if (vm) {
            gen_helper_vnmsac_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vnmsac_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WADDU:
        if (vm) {
            gen_helper_vwaddu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwaddu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WADD:
        if (vm) {
            gen_helper_vwadd_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwadd_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WSUBU:
        if (vm) {
            gen_helper_vwsubu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwsubu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WSUB:
        if (vm) {
            gen_helper_vwsub_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwsub_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WADDUW:
        if (vm) {
            gen_helper_vwaddu_mwv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwaddu_mwv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WADDW:
        if (vm) {
            gen_helper_vwadd_mwv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwadd_mwv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WSUBUW:
        if (vm) {
            gen_helper_vwsubu_mwv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwsubu_mwv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WSUBW:
        if (vm) {
            gen_helper_vwsub_mwv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwsub_mwv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WMULU:
        if (vm) {
            gen_helper_vwmulu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwmulu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WMULSU:
        if (vm) {
            gen_helper_vwmulsu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwmulsu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WMUL:
        if (vm) {
            gen_helper_vwmul_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwmul_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WMACCU:
        if (vm) {
            gen_helper_vwmaccu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwmaccu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WMACC:
        if (vm) {
            gen_helper_vwmacc_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwmacc_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WMACCSU:
        if (vm) {
            gen_helper_vwmaccsu_mvv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vwmaccsu_mvv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free(t_tl);
    tcg_temp_free_i32(t_vd);
    tcg_temp_free_i32(t_vs1);
    tcg_temp_free_i32(t_vs2);
}

static void gen_v_opmvx(DisasContext *dc, uint8_t funct6, int vd, int rs1, int vs2, uint8_t vm)
{
    generate_vill_check(dc);
    TCGv_i32 t_vd, t_vs2;
    TCGv t_tl;
    t_vd = tcg_temp_new_i32();
    t_vs2 = tcg_temp_new_i32();
    t_tl = tcg_temp_new();
    tcg_gen_movi_i32(t_vd, vd);
    tcg_gen_movi_i32(t_vs2, vs2);
    gen_get_gpr(t_tl, rs1);

    switch (funct6) {
    case RISC_V_FUNCT_AADDU:
        if (vm) {
            gen_helper_vaaddu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vaaddu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_AADD:
        if (vm) {
            gen_helper_vaadd_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vaadd_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_ASUBU:
        if (vm) {
            gen_helper_vasubu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vasubu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_ASUB:
        if (vm) {
            gen_helper_vasub_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vasub_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_SLIDE1UP:
        if (vm) {
            gen_helper_vslide1up(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vslide1up_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_SLIDE1DOWN:
        if (vm) {
            gen_helper_vslide1down(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vslide1down_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_RXUNARY0:
        if (vs2 == 0x0 && vm) {
            gen_helper_vmv_sx(cpu_env, t_vd, t_tl);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_DIVU:
        if (vm) {
            gen_helper_vdivu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vdivu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_DIV:
        if (vm) {
            gen_helper_vdiv_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vdiv_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_REMU:
        if (vm) {
            gen_helper_vremu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vremu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_REM:
        if (vm) {
            gen_helper_vrem_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vrem_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_MULHU:
        if (vm) {
            gen_helper_vmulhu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vmulhu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_MUL:
        if (vm) {
            gen_helper_vmul_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vmul_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_MULHSU:
        if (vm) {
            gen_helper_vmulhsu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vmulhsu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_MULH:
        if (vm) {
            gen_helper_vmulh_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vmulh_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_MADD:
        if (vm) {
            gen_helper_vmadd_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vmadd_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_NMSUB:
        if (vm) {
            gen_helper_vnmsub_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vnmsub_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_MACC:
        if (vm) {
            gen_helper_vmacc_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vmacc_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_NMSAC:
        if (vm) {
            gen_helper_vnmsac_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vnmsac_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WADDU:
        if (vm) {
            gen_helper_vwaddu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwaddu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WADD:
        if (vm) {
            gen_helper_vwadd_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwadd_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WSUBU:
        if (vm) {
            gen_helper_vwsubu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwsubu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WSUB:
        if (vm) {
            gen_helper_vwsub_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwsub_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WADDUW:
        if (vm) {
            gen_helper_vwaddu_mwx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwaddu_mwx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WADDW:
        if (vm) {
            gen_helper_vwadd_mwx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwadd_mwx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WSUBUW:
        if (vm) {
            gen_helper_vwsubu_mwx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwsubu_mwx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WSUBW:
        if (vm) {
            gen_helper_vwsub_mwx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwsub_mwx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WMULU:
        if (vm) {
            gen_helper_vwmulu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwmulu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WMULSU:
        if (vm) {
            gen_helper_vwmulsu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwmulsu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WMUL:
        if (vm) {
            gen_helper_vwmul_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwmul_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WMACCU:
        if (vm) {
            gen_helper_vwmaccu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwmaccu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WMACC:
        if (vm) {
            gen_helper_vwmacc_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwmacc_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WMACCUS:
        if (vm) {
            gen_helper_vwmaccus_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwmaccus_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    case RISC_V_FUNCT_WMACCSU:
        if (vm) {
            gen_helper_vwmaccsu_mvx(cpu_env, t_vd, t_vs2, t_tl);
        } else {
            gen_helper_vwmaccsu_mvx_m(cpu_env, t_vd, t_vs2, t_tl);
        }
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free(t_tl);
    tcg_temp_free_i32(t_vd);
    tcg_temp_free_i32(t_vs2);
}

static void gen_v_opfvv(DisasContext *dc, uint8_t funct6, int vd, int vs1, int vs2, uint8_t vm)
{
    generate_vill_check(dc);
    TCGv_i32 t_vd, t_vs2, t_vs1;
    t_vd = tcg_temp_new_i32();
    t_vs2 = tcg_temp_new_i32();
    t_vs1 = tcg_temp_new_i32();
    tcg_gen_movi_i32(t_vd, vd);
    tcg_gen_movi_i32(t_vs2, vs2);
    tcg_gen_movi_i32(t_vs1, vs1);

    switch (funct6) {
    case RISC_V_FUNCT_FADD:
        if (vm) {
            gen_helper_vfadd_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfadd_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FREDSUM:
        if (vm) {
            gen_helper_vfredsum_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfredsum_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FSUB:
        if (vm) {
            gen_helper_vfsub_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfsub_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FREDOSUM:
        if (vm) {
            gen_helper_vfredsum_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfredsum_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FMIN:
        if (vm) {
            gen_helper_vfmin_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfmin_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FREDMIN:
        if (vm) {
            gen_helper_vfredmin_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfredmin_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FMAX:
        if (vm) {
            gen_helper_vfmax_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfmax_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FREDMAX:
        if (vm) {
            gen_helper_vfredmax_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfredmax_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FSGNJ:
        if (vm) {
            gen_helper_vfsgnj_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfsgnj_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FSGNJN:
        if (vm) {
            gen_helper_vfsgnjn_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfsgnjn_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FSGNJX:
        if (vm) {
            gen_helper_vfsgnjx_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfsgnjx_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_WFUNARY0:
        if (vm && vs1 == 0) {
            gen_helper_vfmv_fs(cpu_env, t_vd, t_vs2);
        } else {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        }
        break;
    case RISC_V_FUNCT_FUNARY0:
        switch (vs1) {
        case 0x0:
            if (vm) {
                gen_helper_vfcvt_xuf_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfcvt_xuf_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x1:
            if (vm) {
                gen_helper_vfcvt_xf_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfcvt_xf_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x2:
            if (vm) {
                gen_helper_vfcvt_fxu_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfcvt_fxu_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x3:
            if (vm) {
                gen_helper_vfcvt_fx_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfcvt_fx_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x6:
            if (vm) {
                gen_helper_vfcvt_rtz_xuf_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfcvt_rtz_xuf_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x7:
            if (vm) {
                gen_helper_vfcvt_rtz_xf_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfcvt_rtz_xf_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x8:
            if (vm) {
                gen_helper_vfwcvt_xuf_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfwcvt_xuf_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x9:
            if (vm) {
                gen_helper_vfwcvt_xf_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfwcvt_xf_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0xa:
            if (vm) {
                gen_helper_vfwcvt_fxu_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfwcvt_fxu_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0xb:
            if (vm) {
                gen_helper_vfwcvt_fx_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfwcvt_fx_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0xc:
            if (vm) {
                gen_helper_vfwcvt_ff_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfwcvt_ff_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0xe:
            if (vm) {
                gen_helper_vfwcvt_rtz_xuf_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfwcvt_rtz_xuf_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0xf:
            if (vm) {
                gen_helper_vfwcvt_rtz_xf_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfwcvt_rtz_xf_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x10:
            if (vm) {
                gen_helper_vfncvt_xuf_w(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfncvt_xuf_w_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x11:
            if (vm) {
                gen_helper_vfncvt_xf_w(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfncvt_xf_w_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x12:
            if (vm) {
                gen_helper_vfncvt_fxu_w(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfncvt_fxu_w_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x13:
            if (vm) {
                gen_helper_vfncvt_fx_w(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfncvt_fx_w_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x14:
            if (vm) {
                gen_helper_vfncvt_ff_w(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfncvt_ff_w_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x15:
            if (vm) {
                gen_helper_vfncvt_rod_ff_w(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfncvt_rod_ff_w_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x16:
            if (vm) {
                gen_helper_vfncvt_rtz_xuf_w(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfncvt_rtz_xuf_w_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x17:
            if (vm) {
                gen_helper_vfncvt_rtz_xf_w(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfncvt_rtz_xf_w_m(cpu_env, t_vd, t_vs2);
            }
            break;
        default:
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        break;
    case RISC_V_FUNCT_FUNARY1:
        switch (vs1) {
        case 0x0:
            if (vm) {
                gen_helper_vfsqrt_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfsqrt_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x4:
            if (vm) {
                gen_helper_vfrsqrt7_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfrsqrt7_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x5:
            if (vm) {
                gen_helper_vfrec7_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfrec7_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        case 0x10:
            if (vm) {
                gen_helper_vfclass_v(cpu_env, t_vd, t_vs2);
            } else {
                gen_helper_vfclass_v_m(cpu_env, t_vd, t_vs2);
            }
            break;
        default:
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        break;
    case RISC_V_FUNCT_MFEQ:
        if (vm) {
            gen_helper_vfeq_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfeq_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MFLE:
        if (vm) {
            gen_helper_vfle_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfle_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MFLT:
        if (vm) {
            gen_helper_vflt_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vflt_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_MFNE:
        if (vm) {
            gen_helper_vfne_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfne_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FDIV:
        if (vm) {
            gen_helper_vfdiv_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfdiv_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FMUL:
        if (vm) {
            gen_helper_vfmul_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfmul_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FMADD:
        if (vm) {
            gen_helper_vfmadd_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfmadd_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FNMADD:
        if (vm) {
            gen_helper_vfnmadd_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfnmadd_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FMSUB:
        if (vm) {
            gen_helper_vfmsub_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfmsub_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FNMSUB:
        if (vm) {
            gen_helper_vfnmsub_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfnmsub_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FMACC:
        if (vm) {
            gen_helper_vfmacc_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfmacc_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FNMACC:
        if (vm) {
            gen_helper_vfnmacc_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfnmacc_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FMSAC:
        if (vm) {
            gen_helper_vfmsac_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfmsac_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FNMSAC:
        if (vm) {
            gen_helper_vfnmsac_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfnmsac_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWADD:
        if (vm) {
            gen_helper_vfwadd_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwadd_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWREDSUM:
        if (vm) {
            gen_helper_vfwredsum_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwredsum_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWSUB:
        if (vm) {
            gen_helper_vfwsub_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwsub_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWREDOSUM:
        if (vm) {
            gen_helper_vfwredsum_vs(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwredsum_vs_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWADDW:
        if (vm) {
            gen_helper_vfwadd_wv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwadd_wv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWSUBW:
        if (vm) {
            gen_helper_vfwsub_wv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwsub_wv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWMUL:
        if (vm) {
            gen_helper_vfwmul_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwmul_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWMACC:
        if (vm) {
            gen_helper_vfwmacc_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwmacc_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWNMACC:
        if (vm) {
            gen_helper_vfwnmacc_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwnmacc_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWMSAC:
        if (vm) {
            gen_helper_vfwmsac_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwmsac_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    case RISC_V_FUNCT_FWNMSAC:
        if (vm) {
            gen_helper_vfwnmsac_vv(cpu_env, t_vd, t_vs2, t_vs1);
        } else {
            gen_helper_vfwnmsac_vv_m(cpu_env, t_vd, t_vs2, t_vs1);
        }
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free_i32(t_vd);
    tcg_temp_free_i32(t_vs2);
    tcg_temp_free_i32(t_vs1);
}

static void gen_v_opfvf(DisasContext *dc, uint8_t funct6, int vd, int rs1, int vs2, uint8_t vm)
{
    generate_vill_check(dc);
    TCGv t_vd, t_vs2;
    t_vd = tcg_temp_new_i32();
    t_vs2 = tcg_temp_new_i32();
    tcg_gen_movi_i32(t_vd, vd);
    tcg_gen_movi_i32(t_vs2, vs2);

    switch (funct6) {
    case RISC_V_FUNCT_FADD:
        if (vm) {
            gen_helper_vfadd_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfadd_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FSUB:
        if (vm) {
            gen_helper_vfsub_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfsub_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FMIN:
        if (vm) {
            gen_helper_vfmin_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfmin_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FMAX:
        if (vm) {
            gen_helper_vfmax_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfmax_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FSGNJ:
        if (vm) {
            gen_helper_vfsgnj_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfsgnj_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FSGNJN:
        if (vm) {
            gen_helper_vfsgnjn_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfsgnjn_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FSGNJX:
        if (vm) {
            gen_helper_vfsgnjx_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfsgnjx_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FSLIDE1UP:
        if (vm) {
            gen_helper_vfslide1up(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfslide1up_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FSLIDE1DOWN:
        if (vm) {
            gen_helper_vfslide1down(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfslide1down_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_RFUNARY0:
        if (vm && vs2 == 0) {
                gen_get_fpr(t_vs2, vs2);
                gen_helper_vfmv_sf(cpu_env, t_vd, cpu_fpr[rs1]);
                break;
            }
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    case RISC_V_FUNCT_FMERGE_FMV:
        if (vm) {
            if (vs2) {
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            } else {
                gen_helper_vfmv_vf(cpu_env, t_vd, cpu_fpr[rs1]);
            }
        } else {
            gen_helper_vfmerge_vfm(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_MFEQ:
        if (vm) {
            gen_helper_vfeq_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfeq_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_MFLE:
        if (vm) {
            gen_helper_vfle_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfle_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_MFLT:
        if (vm) {
            gen_helper_vflt_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vflt_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_MFNE:
        if (vm) {
            gen_helper_vfne_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfne_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_MFGT:
        if (vm) {
            gen_helper_vfgt_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfgt_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_MFGE:
        if (vm) {
            gen_helper_vfge_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfge_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FDIV:
        if (vm) {
            gen_helper_vfdiv_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfdiv_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FRDIV:
        if (vm) {
            gen_helper_vfrdiv_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfrdiv_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FMUL:
        if (vm) {
            gen_helper_vfmul_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfmul_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FRSUB:
        if (vm) {
            gen_helper_vfrsub_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfrsub_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FMADD:
        if (vm) {
            gen_helper_vfmadd_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfmadd_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FNMADD:
        if (vm) {
            gen_helper_vfnmadd_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfnmadd_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FMSUB:
        if (vm) {
            gen_helper_vfmsub_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfmsub_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FNMSUB:
        if (vm) {
            gen_helper_vfnmsub_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfnmsub_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FMACC:
        if (vm) {
            gen_helper_vfmacc_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfmacc_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FNMACC:
        if (vm) {
            gen_helper_vfnmacc_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfnmacc_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FMSAC:
        if (vm) {
            gen_helper_vfmsac_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfmsac_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FNMSAC:
        if (vm) {
            gen_helper_vfnmsac_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfnmsac_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FWADD:
        if (vm) {
            gen_helper_vfwadd_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfwadd_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FWSUB:
        if (vm) {
            gen_helper_vfwsub_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfwsub_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FWADDW:
        if (vm) {
            gen_helper_vfwadd_wf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfwadd_wf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FWSUBW:
        if (vm) {
            gen_helper_vfwsub_wf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfwsub_wf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FWMUL:
        if (vm) {
            gen_helper_vfwmul_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfwmul_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FWMACC:
        if (vm) {
            gen_helper_vfwmacc_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfwmacc_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FWNMACC:
        if (vm) {
            gen_helper_vfwnmacc_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfwnmacc_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FWMSAC:
        if (vm) {
            gen_helper_vfwmsac_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfwmsac_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    case RISC_V_FUNCT_FWNMSAC:
        if (vm) {
            gen_helper_vfwnmsac_vf(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        } else {
            gen_helper_vfwnmsac_vf_m(cpu_env, t_vd, t_vs2, cpu_fpr[rs1]);
        }
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
    tcg_temp_free_i32(t_vd);
    tcg_temp_free_i32(t_vs2);
}
#endif  // HOST_LONG_BITS != 32

static void gen_v(DisasContext *dc, uint32_t opc, int rd, int rs1, int rs2, int imm)
{
// Vector helpers require 128-bit ints which aren't supported on 32-bit hosts.
#if HOST_LONG_BITS == 32
    tlib_abort("Vector extension isn't available on 32-bit hosts.");
#else
    if (!ensure_extension(dc, RISCV_FEATURE_RVV)) {
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        return;
    }
    uint8_t funct6 = extract32(dc->opcode, 26, 6);
    uint8_t vm = extract32(dc->opcode, 25, 1);

    // DataLab: variable opclass
    uint32_t opclass = 0;

    switch (opc) {
    case OPC_RISC_V_IVV:
        opclass = DL_RISC_V_IVV;
        gen_v_opivv(dc, funct6, rd, rs1, rs2, vm);
        break;
    case OPC_RISC_V_FVV:
        opclass = DL_RISC_V_FVV;
        gen_v_opfvv(dc, funct6, rd, rs1, rs2, vm);
        break;
    case OPC_RISC_V_MVV:
        opclass = DL_RISC_V_MVV;
        gen_v_opmvv(dc, funct6, rd, rs1, rs2, vm);
        break;
    case OPC_RISC_V_IVI:
        opclass = DL_RISC_V_IVI;
        gen_v_opivi(dc, funct6, rd, rs1, rs2, vm);
        break;
    case OPC_RISC_V_IVX:
        opclass = DL_RISC_V_IVX;
        gen_v_opivx(dc, funct6, rd, rs1, rs2, vm);
        break;
    case OPC_RISC_V_FVF:
        opclass = DL_RISC_V_FVF;
        gen_v_opfvf(dc, funct6, rd, rs1, rs2, vm);
        break;
    case OPC_RISC_V_MVX:
        opclass = DL_RISC_V_MVX;
        gen_v_opmvx(dc, funct6, rd, rs1, rs2, vm);
        break;
    case OPC_RISC_V_CFG:
        gen_v_cfg(dc, MASK_OP_V_CFG(dc->opcode), rd, rs1, rs2, imm);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }

#ifdef DL_TRACE_ARITH
    if (opc != OPC_RISC_V_CFG) { // DataLab: helper call
        TCGv_i32 topc = tcg_const_i32(opc);
        TCGv_i32 topclass = tcg_const_i32(opclass);
        gen_sync_pc(dc);
        gen_helper_log_inst_arith(cpu_env, topc, topclass);
        tcg_temp_free(topc);
        tcg_temp_free(topclass);
    }
#else
    TCGv_i32 topclass = tcg_const_i32(opclass);
    tcg_temp_free(topclass);
#endif

    tcg_gen_movi_tl(cpu_vstart, 0);
#endif  // HOST_LONG_BITS != 32
}

static void decode_RV32_64C0(DisasContext *dc)
{
    uint8_t funct3 = extract32(dc->opcode, 13, 3);
    uint8_t rd_rs2 = GET_C_RS2S(dc->opcode);
    uint8_t rs1s = GET_C_RS1S(dc->opcode);

    switch (funct3) {
    case 0:
        /* illegal */
        if (dc->opcode == 0) {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        } else {
            /* C.ADDI4SPN -> addi rd', x2, zimm[9:2]*/
            gen_arith_imm(dc, OPC_RISC_ADDI, rd_rs2, 2, GET_C_ADDI4SPN_IMM(dc->opcode));
        }
        break;
    case 1:
        /* C.FLD -> fld rd', offset[7:3](rs1')*/
        gen_fp_load(dc, OPC_RISC_FLD, rd_rs2, rs1s, GET_C_LD_IMM(dc->opcode));
        /* C.LQ(RV128) */
        break;
    case 2:
        /* C.LW -> lw rd', offset[6:2](rs1') */
        gen_load(dc, OPC_RISC_LW, rd_rs2, rs1s, GET_C_LW_IMM(dc->opcode));
        break;
    case 3:
#if defined(TARGET_RISCV64)
        /* C.LD(RV64/128) -> ld rd', offset[7:3](rs1')*/
        gen_load(dc, OPC_RISC_LD, rd_rs2, rs1s, GET_C_LD_IMM(dc->opcode));
#else
        /* C.FLW (RV32) -> flw rd', offset[6:2](rs1')*/
        gen_fp_load(dc, OPC_RISC_FLW, rd_rs2, rs1s, GET_C_LW_IMM(dc->opcode));
#endif
        break;
    case 4:
        /* reserved */
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    case 5:
        /* C.FSD(RV32/64) -> fsd rs2', offset[7:3](rs1') */
        gen_fp_store(dc, OPC_RISC_FSD, rs1s, rd_rs2, GET_C_LD_IMM(dc->opcode));
        /* C.SQ (RV128) */
        break;
    case 6:
        /* C.SW -> sw rs2', offset[6:2](rs1')*/
        gen_store(dc, OPC_RISC_SW, rs1s, rd_rs2, GET_C_LW_IMM(dc->opcode));
        break;
    case 7:
#if defined(TARGET_RISCV64)
        /* C.SD (RV64/128) -> sd rs2', offset[7:3](rs1')*/
        gen_store(dc, OPC_RISC_SD, rs1s, rd_rs2, GET_C_LD_IMM(dc->opcode));
#else
        /* C.FSW (RV32) -> fsw rs2', offset[6:2](rs1')*/
        gen_fp_store(dc, OPC_RISC_FSW, rs1s, rd_rs2, GET_C_LW_IMM(dc->opcode));
#endif
        break;
    }
}

static void decode_RV32_64C1(CPUState *env, DisasContext *dc)
{
    uint8_t funct3 = extract32(dc->opcode, 13, 3);
    uint8_t rd_rs1 = GET_C_RS1(dc->opcode);
    uint8_t rs1s, rs2s;
    uint8_t funct2;
    target_long imm;

    switch (funct3) {
    case 0:
        /* C.ADDI -> addi rd, rd, nzimm[5:0] */
        gen_arith_imm(dc, OPC_RISC_ADDI, rd_rs1, rd_rs1, GET_C_IMM(dc->opcode));
        break;
    case 1:
#if defined(TARGET_RISCV64)
        /* C.ADDIW (RV64/128) -> addiw rd, rd, imm[5:0]*/
        if (!rd_rs1) { /* ISA V20191213: Reserved when rd == 0 */
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        } else {
            gen_arith_imm(dc, OPC_RISC_ADDIW, rd_rs1, rd_rs1, GET_C_IMM(dc->opcode));
        }
#else
        /* C.JAL(RV32) -> jal x1, offset[11:1] */
        gen_jal(env, dc, 1, GET_C_J_IMM(dc->opcode));
#endif
        break;
    case 2:
        /* C.LI -> addi rd, x0, imm[5:0]*/
        gen_arith_imm(dc, OPC_RISC_ADDI, rd_rs1, 0, GET_C_IMM(dc->opcode));
        break;
    case 3:
        if (rd_rs1 == 2) {
            imm = GET_C_ADDI16SP_IMM(dc->opcode);
            /* C.ADDI16SP -> addi x2, x2, nzimm[9:4]*/
            if (!imm) { /* ISA V20191213: Reserved when nzimm == 0 */
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            } else {
                gen_arith_imm(dc, OPC_RISC_ADDI, 2, 2, imm);
            }
        } else if (rd_rs1 != 0) {
            imm = GET_C_IMM(dc->opcode);
            /* C.LUI (rs1/rd =/= {0,2}) -> lui rd, nzimm[17:12]*/
            if (!imm) { /* ISA V20191213: Reserved when nzimm == 0 */
                kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            } else {
                get_set_gpr_imm(rd_rs1, imm << 12);
            }
        }
        break;
    case 4:
        funct2 = extract32(dc->opcode, 10, 2);
        rs1s = GET_C_RS1S(dc->opcode);
        switch (funct2) {
        case 0: /* C.SRLI(RV32) -> srli rd', rd', shamt[5:0] */
            gen_arith_imm(dc, OPC_RISC_SHIFT_RIGHT_I, rs1s, rs1s, GET_C_ZIMM(dc->opcode));
            /* C.SRLI64(RV128) */
            break;
        case 1:
            /* C.SRAI -> srai rd', rd', shamt[5:0]*/
            gen_arith_imm(dc, OPC_RISC_SHIFT_RIGHT_I, rs1s, rs1s, GET_C_ZIMM(dc->opcode) | 0x400);
            /* C.SRAI64(RV128) */
            break;
        case 2:
            /* C.ANDI -> andi rd', rd', imm[5:0]*/
            gen_arith_imm(dc, OPC_RISC_ANDI, rs1s, rs1s, GET_C_IMM(dc->opcode));
            break;
        case 3:
            funct2 = extract32(dc->opcode, 5, 2);
            rs2s = GET_C_RS2S(dc->opcode);
            switch (funct2) {
            case 0:
                /* C.SUB -> sub rd', rd', rs2' */
                if (extract32(dc->opcode, 12, 1) == 0) {
                    gen_arith(dc, OPC_RISC_SUB, rs1s, rs1s, rs2s);
                }
#if defined(TARGET_RISCV64)
                else {
                    gen_arith(dc, OPC_RISC_SUBW, rs1s, rs1s, rs2s);
                }
#endif
                break;
            case 1:
                /* C.XOR -> xor rs1', rs1', rs2' */
                if (extract32(dc->opcode, 12, 1) == 0) {
                    gen_arith(dc, OPC_RISC_XOR, rs1s, rs1s, rs2s);
                }
#if defined(TARGET_RISCV64)
                else {
                    /* C.ADDW (RV64/128) */
                    gen_arith(dc, OPC_RISC_ADDW, rs1s, rs1s, rs2s);
                }
#endif
                break;
            case 2:
                /* C.OR -> or rs1', rs1', rs2' */
                gen_arith(dc, OPC_RISC_OR, rs1s, rs1s, rs2s);
                break;
            case 3:
                /* C.AND -> and rs1', rs1', rs2' */
                gen_arith(dc, OPC_RISC_AND, rs1s, rs1s, rs2s);
                break;
            }
            break;
        }
        break;
    case 5:
        /* C.J -> jal x0, offset[11:1]*/
        gen_jal(env, dc, 0, GET_C_J_IMM(dc->opcode));
        break;
    case 6:
        /* C.BEQZ -> beq rs1', x0, offset[8:1]*/
        rs1s = GET_C_RS1S(dc->opcode);
        gen_branch(env, dc, OPC_RISC_BEQ, rs1s, 0, GET_C_B_IMM(dc->opcode));
        break;
    case 7:
        /* C.BNEZ -> bne rs1', x0, offset[8:1]*/
        rs1s = GET_C_RS1S(dc->opcode);
        gen_branch(env, dc, OPC_RISC_BNE, rs1s, 0, GET_C_B_IMM(dc->opcode));
        break;
    }
}

static void decode_RV32_64C2(CPUState *env, DisasContext *dc)
{
    uint8_t rd, rs2;
    uint8_t funct3 = extract32(dc->opcode, 13, 3);

    rd = GET_RD(dc->opcode);

    switch (funct3) {
    case 0: /* C.SLLI -> slli rd, rd, shamt[5:0]
               C.SLLI64 -> */
        gen_arith_imm(dc, OPC_RISC_SLLI, rd, rd, GET_C_ZIMM(dc->opcode));
        break;
    case 1: /* C.FLDSP(RV32/64DC) -> fld rd, offset[8:3](x2) */
        gen_fp_load(dc, OPC_RISC_FLD, rd, 2, GET_C_LDSP_IMM(dc->opcode));
        break;
    case 2: /* C.LWSP -> lw rd, offset[7:2](x2) */
        if (!rd) { /* ISA V20191213: Reserved when rd == 0 */
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        } else {
            gen_load(dc, OPC_RISC_LW, rd, 2, GET_C_LWSP_IMM(dc->opcode));
        }
        break;
    case 3:
#if defined(TARGET_RISCV64)
        /* C.LDSP(RVC64) -> ld rd, offset[8:3](x2) */
        if (!rd) { /* ISA V20191213: Reserved when rd == 0 */
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
            break;
        }
        gen_load(dc, OPC_RISC_LD, rd, 2, GET_C_LDSP_IMM(dc->opcode));
#else
        /* C.FLWSP(RV32FC) -> flw rd, offset[7:2](x2) */
        gen_fp_load(dc, OPC_RISC_FLW, rd, 2, GET_C_LWSP_IMM(dc->opcode));
#endif
        break;
    case 4:
        rs2 = GET_C_RS2(dc->opcode);

        if (extract32(dc->opcode, 12, 1) == 0) {
            if (rs2 == 0) {
                /* C.JR -> jalr x0, rs1, 0*/
                if (!rd) {/* ISA V20191213: Reserved when rd == 0 */
                    kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
                } else {
                    gen_jalr(env, dc, OPC_RISC_JALR, 0, rd, 0);
                }
            } else {
                /* C.MV -> add rd, x0, rs2 */
                gen_arith(dc, OPC_RISC_ADD, rd, 0, rs2);
            }
        } else {
            if (rd == 0) {
                /* C.EBREAK -> ebreak*/
                gen_system(dc, OPC_RISC_ECALL, 0, 0, 0x1, 0);
            } else {
                if (rs2 == 0) {
                    /* C.JALR -> jalr x1, rs1, 0*/
                    gen_jalr(env, dc, OPC_RISC_JALR, 1, rd, 0);
                } else {
                    /* C.ADD -> add rd, rd, rs2 */
                    gen_arith(dc, OPC_RISC_ADD, rd, rd, rs2);
                }
            }
        }
        break;
    case 5:
        /* C.FSDSP -> fsd rs2, offset[8:3](x2)*/
        gen_fp_store(dc, OPC_RISC_FSD, 2, GET_C_RS2(dc->opcode), GET_C_SDSP_IMM(dc->opcode));
        /* C.SQSP */
        break;
    case 6: /* C.SWSP -> sw rs2, offset[7:2](x2)*/
        gen_store(dc, OPC_RISC_SW, 2, GET_C_RS2(dc->opcode), GET_C_SWSP_IMM(dc->opcode));
        break;
    case 7:
#if defined(TARGET_RISCV64)
        /* C.SDSP(Rv64/128) -> sd rs2, offset[8:3](x2)*/
        gen_store(dc, OPC_RISC_SD, 2, GET_C_RS2(dc->opcode), GET_C_SDSP_IMM(dc->opcode));
#else
        /* C.FSWSP(RV32) -> fsw rs2, offset[7:2](x2) */
        gen_fp_store(dc, OPC_RISC_FSW, 2, GET_C_RS2(dc->opcode), GET_C_SWSP_IMM(dc->opcode));
#endif
        break;
    }
}

static void decode_RV32_64C(CPUState *env, DisasContext *dc)
{
    uint8_t op = extract32(dc->opcode, 0, 2);

    switch (op) {
    case 0:
        decode_RV32_64C0(dc);
        break;
    case 1:
        decode_RV32_64C1(env, dc);
        break;
    case 2:
        decode_RV32_64C2(env, dc);
        break;
    }
}

static void decode_RV32_64G(CPUState *env, DisasContext *dc)
{
    int rs1;
    int rs2;
    int rd;
    uint32_t rm;
    uint32_t op;
    target_long imm;

    /* We do not do misaligned address check here: the address should never be
     * misaligned at this point. Instructions that set PC must do the check,
     * since epc must be the address of the instruction that caused us to
     * perform the misaligned instruction fetch */

    op = MASK_OP_MAJOR(dc->opcode);
    rs1 = GET_RS1(dc->opcode);
    rs2 = GET_RS2(dc->opcode);
    rd = GET_RD(dc->opcode);
    imm = GET_IMM(dc->opcode);
    rm = GET_RM(dc->opcode);

    switch (op) {
    case OPC_RISC_LUI:
        if (rd == 0) {
            break; /* NOP */
        }
        get_set_gpr_imm(rd, sextract64(dc->opcode, 12, 20) << 12);
        break;
    case OPC_RISC_AUIPC:
        if (rd == 0) {
            break; /* NOP */
        }
        get_set_gpr_imm(rd, (sextract64(dc->opcode, 12, 20) << 12) + dc->base.pc);
        break;
    case OPC_RISC_JAL:
        imm = GET_JAL_IMM(dc->opcode);
        gen_jal(env, dc, rd, imm);
        break;
    case OPC_RISC_JALR:
        gen_jalr(env, dc, MASK_OP_JALR(dc->opcode), rd, rs1, imm);
        break;
    case OPC_RISC_BRANCH:
        gen_branch(env, dc, MASK_OP_BRANCH(dc->opcode), rs1, rs2, GET_B_IMM(dc->opcode));
        break;
    case OPC_RISC_LOAD:
        /* Illegal, RV128I is not supported yet */
        if (MASK_OP_LOAD(dc->opcode) == OPC_RISC_LDU) {
            kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        } else {
            gen_load(dc, MASK_OP_LOAD(dc->opcode), rd, rs1, imm);
        }
        break;
    case OPC_RISC_STORE:
        gen_store(dc, MASK_OP_STORE(dc->opcode), rs1, rs2, GET_STORE_IMM(dc->opcode));
        break;
    case OPC_RISC_ARITH_IMM:
#if defined(TARGET_RISCV64)
    case OPC_RISC_ARITH_IMM_W:
#endif
        if (rd == 0) {
            break; /* NOP */
        }
        gen_arith_imm(dc, MASK_OP_ARITH_IMM(dc->opcode), rd, rs1, imm);
        break;
    case OPC_RISC_ARITH:
#if defined(TARGET_RISCV64)
    case OPC_RISC_ARITH_W:
#endif
        if (rd == 0) {
            break; /* NOP */
        }
        gen_arith(dc, MASK_OP_ARITH(dc->opcode), rd, rs1, rs2);
        break;
    case OPC_RISC_FP_LOAD:
        if (rm - 1 < 4) {
            gen_fp_load(dc, MASK_OP_FP_LOAD(dc->opcode), rd, rs1, imm);
        } else {
            gen_v_load(dc, MASK_OP_V_LOAD(dc->opcode), imm >> 5, rd, rs1, rs2, rm);
        }
        break;
    case OPC_RISC_FP_STORE:
        if (rm - 1 < 4) {
            gen_fp_store(dc, MASK_OP_FP_STORE(dc->opcode), rs1, rs2, GET_STORE_IMM(dc->opcode));
        } else {
            gen_v_store(dc, MASK_OP_V_STORE(dc->opcode), imm >> 5, rd, rs1, rs2, rm);
        }
        break;
    case OPC_RISC_ATOMIC:
        gen_atomic(env, dc, MASK_OP_ATOMIC(dc->opcode), rd, rs1, rs2);
        break;
    case OPC_RISC_FMADD:
        gen_fp_fmadd(dc, MASK_OP_FP_FMADD(dc->opcode), rd, rs1, rs2, GET_RS3(dc->opcode), GET_RM(dc->opcode));
        break;
    case OPC_RISC_FMSUB:
        gen_fp_fmsub(dc, MASK_OP_FP_FMSUB(dc->opcode), rd, rs1, rs2, GET_RS3(dc->opcode), GET_RM(dc->opcode));
        break;
    case OPC_RISC_FNMSUB:
        gen_fp_fnmsub(dc, MASK_OP_FP_FNMSUB(dc->opcode), rd, rs1, rs2, GET_RS3(dc->opcode), GET_RM(dc->opcode));
        break;
    case OPC_RISC_FNMADD:
        gen_fp_fnmadd(dc, MASK_OP_FP_FNMADD(dc->opcode), rd, rs1, rs2, GET_RS3(dc->opcode), GET_RM(dc->opcode));
        break;
    case OPC_RISC_FP_ARITH:
        gen_fp_arith(dc, MASK_OP_FP_ARITH(dc->opcode), rd, rs1, rs2, GET_RM(dc->opcode));
        break;
    case OPC_RISC_SYNCH:
        gen_synch(dc, MASK_OP_FENCE(dc->opcode));
        break;
    case OPC_RISC_SYSTEM:
        gen_system(dc, MASK_OP_SYSTEM(dc->opcode), rd, rs1, rs2, GET_FUNCT12(dc->opcode));
        break;
    case OPC_RISC_V:
        gen_v(dc, MASK_OP_V(dc->opcode), rd, rs1, rs2, imm);
        break;
    default:
        kill_unknown(dc, RISCV_EXCP_ILLEGAL_INST);
        break;
    }
}

static int disas_insn(CPUState *env, DisasContext *dc)
{
    dc->opcode = ldq_code(dc->base.pc);
    /* handle custom instructions */
    int i;
    for (i = 0; i < env->custom_instructions_count; i++) {
        custom_instruction_descriptor_t *ci = &env->custom_instructions[i];

        if ((dc->opcode & ci->mask) == ci->pattern) {
            dc->npc = dc->base.pc + ci->length;

            if (env->count_opcodes) {
                generate_opcode_count_increment(env, dc->opcode);
            }

            TCGv_i64 id = tcg_const_i64(ci->id);
            TCGv_i64 opcode = tcg_const_i64(dc->opcode & ((1ULL << (8 * ci->length)) - 1));
            TCGv_i32 pc_modified = tcg_temp_new_i32();

            /* DataLab: instruction counter */
            gen_helper_inst_ctr();

            gen_sync_pc(dc);
            gen_helper_handle_custom_instruction(pc_modified, id, opcode);

            int exit_tb_label = gen_new_label();
            tcg_gen_brcondi_i64(TCG_COND_EQ, pc_modified, 1, exit_tb_label);

            // this is executed conditionally - only if `handle_custom_instruction` returns 0
            // otherwise `cpu_pc` points to a proper value and should not be overwritten by `dc->base.pc`
            dc->base.pc = dc->npc;
            gen_sync_pc(dc);

            gen_set_label(exit_tb_label);
            gen_exit_tb_no_chaining(dc->base.tb);
            dc->base.is_jmp = DISAS_BRANCH;

            tcg_temp_free_i64(id);
            tcg_temp_free_i64(opcode);
            tcg_temp_free_i64(pc_modified);

            // custom instruction인 경우 custom instruction 처리하고 return
            return ci->length;
        }
    }

    int instruction_length = decode_instruction_length(dc->opcode);
    // Custom instructions with length up to 64 bits are handled above,
    // but standard RISC-V instructions currently are no longer than 32 bits.
    if (instruction_length == 0 || instruction_length > 4) {
        tlib_printf(LOG_LEVEL_ERROR, "Unsupported instruction length: %d bits. PC: 0x%llx, opcode: 0x%0*llx", 
                    8 * instruction_length , dc->base.pc,  /* padding */ 2 * instruction_length, format_opcode(dc->opcode, instruction_length));
    }
    // Default to 32-bit if encountered an instruction of unsupported length.
    // Allow for an exception to be raised while decoding.
    instruction_length = instruction_length == 2 ? 2 : 4;

    int is_compressed = instruction_length == 2;
    if (is_compressed && !ensure_extension(dc, RISCV_FEATURE_RVC)) {
        return 0;
    }

    // Clear upper bits, leaves only the instruction to be decoded.
    dc->opcode = extract64(dc->opcode, 0, instruction_length * 8);

    /* check for compressed insn */
    dc->npc = dc->base.pc + instruction_length;

    if (env->count_opcodes) {
        generate_opcode_count_increment(env, dc->opcode);
    }

    /* DataLab: instruction counter */
    gen_helper_inst_ctr();

    if (is_compressed) {
        decode_RV32_64C(env, dc);
    } else {
        decode_RV32_64G(env, dc);
    }

    if(unlikely(env->are_post_opcode_execution_hooks_enabled))
    {
        for(int index = 0; index < env->post_opcode_execution_hooks_count; index++)
        {
            opcode_hook_mask_t opcode_def = env->post_opcode_execution_hook_masks[index];
            if((dc->opcode & opcode_def.mask) == opcode_def.value)
            {
                gen_sync_pc(dc);
                TCGv_i32 hook_id = tcg_const_i32(index);
                gen_helper_handle_post_opcode_execution_hook(hook_id, cpu_pc);
                tcg_temp_free_i32(hook_id);
                break;
            }
        }
    }

    dc->base.pc = dc->npc;
    return instruction_length;
}

void setup_disas_context(DisasContextBase *dc, CPUState *env)
{
    dc->mem_idx = cpu_mmu_index(env);
}

int gen_breakpoint(DisasContextBase *base, CPUBreakpoint *bp)
{
    DisasContext *dc = (DisasContext *)base;
    generate_exception(dc, EXCP_DEBUG);
    /* Advance PC so that clearing the breakpoint will
       invalidate this TB.  */
    dc->base.pc += 4;
    return 1;
}

int gen_intermediate_code(CPUState *env, DisasContextBase *base)
{
    tcg_gen_insn_start(base->pc);

    base->tb->size += disas_insn(env, (DisasContext *)base);

    if ((base->pc - (base->tb->pc & TARGET_PAGE_MASK)) >= TARGET_PAGE_SIZE) {
        return 0;
    }

    return 1;
}

uint32_t gen_intermediate_code_epilogue(CPUState *env, DisasContextBase *base)
{
    DisasContext *dc = (DisasContext *)base;
    switch (dc->base.is_jmp) {
    case DISAS_NONE:     /* handle end of page - DO NOT CHAIN. See gen_goto_tb. */
        gen_sync_pc(dc);
        gen_exit_tb_no_chaining(dc->base.tb);
        break;
    case DISAS_NEXT:
    case DISAS_STOP:
        gen_goto_tb(dc, 0, dc->base.pc);
        break;
    case DISAS_BRANCH:     /* ops using DISAS_BRANCH generate own exit seq */
        break;
    }
    return 0;
}

void restore_state_to_opc(CPUState *env, TranslationBlock *tb, target_ulong *data)
{
    env->pc = data[0];
}

void cpu_set_nmi(CPUState *env, int number, target_ulong mcause)
{
    if (number < 0 || number >= env->nmi_length) {
        tlib_abortf("NMI index %d not valid in cpu with nmi_length = %d", number, env->nmi_length);
    } else {
        env->nmi_pending |= (1 << number);
        env->nmi_mcause[number] = mcause;
        set_interrupt_pending(env, CPU_INTERRUPT_HARD);
    }
}

void cpu_reset_nmi(CPUState *env, int number)
{
    env->nmi_pending &= ~(1 << number);
}

int process_interrupt(int interrupt_request, CPUState *env)
{
    /*According to the debug spec draft, the debug mode implies all interrupts are masked (even NMI)
       / and the WFI acts as NOP. */
    if (tlib_is_in_debug_mode()) {
        return 0;
    }
    if (interrupt_request & (CPU_INTERRUPT_HARD | RISCV_CPU_INTERRUPT_CLIC)) {
        int interruptno = riscv_cpu_hw_interrupts_pending(env);
        if (env->nmi_pending > NMI_NONE) {
            do_interrupt(env);
            return 1;
        } else if (interruptno != EXCP_NONE) {
            env->exception_index = RISCV_EXCP_INT_FLAG | interruptno;
            do_interrupt(env);
            return 1;
        }
    }
    return 0;
}

//TODO: These empty implementations are required due to problems with weak attribute.
//Remove this after #7035.
void cpu_exec_epilogue(CPUState *env)
{
}

void cpu_exec_prologue(CPUState *env)
{
}

// void dl_log_inst(CPUState *env) {
//     DLInstDecodeTbl *t = &instDecodeTbl;

//     //uint32_t pc = (uint32_t)CPU_PC(env);
//     uint32_t pc = (uint32_t)env->pc;
//     uint32_t idx = dl_calc_tbl_idx(pc);

//     uint32_t opc = t->tbl[idx].opc;
//     uint16_t rs1 = t->tbl[idx].rs1;
//     uint16_t imm = t->tbl[idx].imm;
//     uint32_t imm32 = (imm >> 15) ? 0xffff0000 | (uint32_t)imm : (uint32_t)imm;
//     uint32_t rs1value = dl_get_reg_value32(env, rs1);
//     ...
// }
//

// uint32_t dl_get_opc(uint32_t pc) {
//     uint32_t idx = dl_calc_tbl_idx(pc);
//     uint32_t opc = instDecodeTbl.tbl[idx].opc;
//     return opc;
// }

void dl_inst_ctr_inc(void) {
    instStat.instCtr++;
}

void dl_put_opc(uint32_t opc) {
    char *opcStr = "unknown";
    uint32_t instCnt = 0;

    switch (opc) {
    /* scalar integer load */
    case OPC_RISC_LB:
        instCnt = (++instStat.lbCnt);
        opcStr = "lb";
        break;
    case OPC_RISC_LH:
        instCnt = (++instStat.lhCnt);
        opcStr = "lh";
        break;
    case OPC_RISC_LW:
        instCnt = (++instStat.lwCnt);
        opcStr = "lw";
        break;
    case OPC_RISC_LD:
        instCnt = (++instStat.ldCnt);
        opcStr = "ld";
        break;
    case OPC_RISC_LBU:
        instCnt = (++instStat.lbuCnt);
        opcStr = "lbu";
        break;
    case OPC_RISC_LHU:
        instCnt = (++instStat.lhuCnt);
        opcStr = "lhu";
        break;
    case OPC_RISC_LWU:
        instCnt = (++instStat.lwuCnt);
        opcStr = "lwu";
        break;
    /* scalar integer store */
    case OPC_RISC_SB:
        instCnt = (++instStat.sbCnt);
        opcStr = "sb";
        break;
    case OPC_RISC_SH:
        instCnt = (++instStat.shCnt);
        opcStr = "sh";
        break;
    case OPC_RISC_SW:
        instCnt = (++instStat.swCnt);
        opcStr = "sw";
        break;
    case OPC_RISC_SD:
        instCnt = (++instStat.sdCnt);
        opcStr = "sd";
        break;
    /* scalar fp load */
    case OPC_RISC_FLH:
        instCnt = (++instStat.flhCnt);
        opcStr = "flh";
        break;
    case OPC_RISC_FLW:
        instCnt = (++instStat.flwCnt);
        opcStr = "flw";
        break;
    case OPC_RISC_FLD:
        instCnt = (++instStat.fldCnt);
        opcStr = "fld";
        break;
    /* scalar fp store */
    case OPC_RISC_FSH:
        instCnt = (++instStat.fshCnt);
        opcStr = "fsh";
        break;
    case OPC_RISC_FSW:
        instCnt = (++instStat.fswCnt);
        opcStr = "fsw";
        break;
    case OPC_RISC_FSD:
        instCnt = (++instStat.fsdCnt);
        opcStr = "fsd";
        break;
    default:
        opcStr = "unknown";
        instCnt = (++instStat.unknownCnt);
        break;
    }
    fprintf(logfp, "[%lu] %s(=%04x)/%u: ", instStat.instCtr, opcStr, opc, instCnt);
}

uint8_t dl_put_opc_bin(uint32_t opc) {
    //uint32_t instCnt = 0;
    uint32_t opResult = 0; // zero: normal, non-zero: unknown or custom
    DLTraceCompactMem trace = { 0, };
    //trace.lower.opType = 0; // mem (traceV1, deprecated)
    switch (opc) {
    /* scalar integer load */
    case OPC_RISC_LB:
        // instCnt = (++instStat.lbCnt);
        trace.lower.opType = 0;         // load (traceV2)
        trace.lower.dataType = 0;       // int
        trace.lower.operandSize = 0;    // 8
        ++instStat.lbCnt;
        break;
    case OPC_RISC_LH:
        // instCnt = (++instStat.lhCnt);
        trace.lower.opType = 0;         // load
        trace.lower.dataType = 0;       // int
        trace.lower.operandSize = 1;    // 16
        ++instStat.lhCnt;
        break;
    case OPC_RISC_LW:
        // instCnt = (++instStat.lwCnt);
        trace.lower.opType = 0;         // load
        trace.lower.dataType = 0;       // int
        trace.lower.operandSize = 2;    // 32
        ++instStat.lwCnt;
        break;
    case OPC_RISC_LD:
        // instCnt = (++instStat.ldCnt);
        trace.lower.opType = 0;         // load
        trace.lower.dataType = 0;       // int
        trace.lower.operandSize = 3;    // 64
        ++instStat.ldCnt;
        break;
    case OPC_RISC_LBU:
        // instCnt = (++instStat.lbuCnt);
        trace.lower.opType = 0;         // load
        trace.lower.dataType = 1;       // uint (traceV2)
        trace.lower.operandSize = 0;    // 8
        ++instStat.lbuCnt;
        break;
    case OPC_RISC_LHU:
        // instCnt = (++instStat.lhuCnt);
        trace.lower.opType = 0;         // load
        trace.lower.dataType = 1;       // uint
        trace.lower.operandSize = 1;    // 16
        ++instStat.lhuCnt;
        break;
    case OPC_RISC_LWU:
        // instCnt = (++instStat.lwuCnt);
        trace.lower.opType = 0;         // load
        trace.lower.dataType = 1;       // uint
        trace.lower.operandSize = 2;    // 32
        ++instStat.lwuCnt;
        break;
    /* scalar integer store */
    case OPC_RISC_SB:
        // instCnt = (++instStat.sbCnt);
        trace.lower.opType = 1;         // store (traceV2)
        trace.lower.dataType = 0;       // int
        trace.lower.operandSize = 0;    // 8
        ++instStat.sbCnt;
        break;
    case OPC_RISC_SH:
        // instCnt = (++instStat.shCnt);
        trace.lower.opType = 1;         // store (traceV2)
        trace.lower.dataType = 0;       // int
        trace.lower.operandSize = 1;   // 16
        ++instStat.shCnt;
        break;
    case OPC_RISC_SW:
        // instCnt = (++instStat.swCnt);
        trace.lower.opType = 1;         // store (traceV2)
        trace.lower.dataType = 0;       // int
        trace.lower.operandSize = 2;    // 32
        ++instStat.swCnt;
        break;
    case OPC_RISC_SD:
        // instCnt = (++instStat.sdCnt);
        trace.lower.opType = 1;         // store (traceV2)
        trace.lower.dataType = 0;       // int
        trace.lower.operandSize = 3;    // 64
        ++instStat.sdCnt;
        break;
    /* scalar fp load */
    case OPC_RISC_FLH:
        // instCnt = (++instStat.flhCnt);
        trace.lower.opType = 0;         // load (traceV2)
        trace.lower.dataType = 2;       // fp (traceV2)
        trace.lower.operandSize = 1;    // FP16
        ++instStat.flhCnt;
        break;
    case OPC_RISC_FLW:
        // instCnt = (++instStat.flwCnt);
        trace.lower.opType = 0;         // load
        trace.lower.dataType = 2;       // fp
        trace.lower.operandSize = 2;    // FP32
        ++instStat.flwCnt;
        break;
    case OPC_RISC_FLD:
        // instCnt = (++instStat.fldCnt);
        trace.lower.opType = 0;         // load
        trace.lower.dataType = 2;       // fp
        trace.lower.operandSize = 3;    // FP64
        ++instStat.fldCnt;
        break;
    /* scalar fp store */
    case OPC_RISC_FSH:
        // instCnt = (++instStat.fshCnt);
        trace.lower.opType = 1;         // store (traceV2)
        trace.lower.dataType = 2;       // fp
        trace.lower.operandSize = 1;    // FP16
        ++instStat.fshCnt;
        break;
    case OPC_RISC_FSW:
        // instCnt = (++instStat.fswCnt);
        trace.lower.opType = 1;         // store
        trace.lower.dataType = 2;       // fp
        trace.lower.operandSize = 2;    // FP32
        ++instStat.fswCnt;
        break;
    case OPC_RISC_FSD:
        // instCnt = (++instStat.fsdCnt);
        trace.lower.opType = 1;         // store
        trace.lower.dataType = 2;       // fp
        trace.lower.operandSize = 3;    // FP64
        ++instStat.fsdCnt;
        break;
    default:
        // instCnt = (++instStat.unknownCnt);
        trace.lower.opType = 3;         // unknown/custom (traceV2)
        // trace.lower.opType = 1;
        // trace.lower.dataType = 0b111;
        // trace.lower.operandSize = 0b1111;
        opResult = DL_RISC_UNKNOWN;
        ++instStat.unknownCnt;
        break;
    }
    trace.instCtr = instStat.instCtr;
    fwrite(&trace.lower, sizeof(DLTraceLow), 1, logfp);
    fwrite(&trace.instCtr, sizeof(uint64_t), 1, logfp);
    return opResult;
}

void dl_put_opc_arith(uint32_t opc, uint32_t opclass) {
#ifdef DL_TRACE_HUMAN_READABLE
    char *opcStr = "unknown";
    uint32_t instCnt = 0;

    if (opclass == DL_RISC_ARITH_IMM) {
        opcStr = "arithimm";
        instCnt = (++instStat.arithImmCnt);
    }
    else if (opclass == DL_RISC_ARITH) {
        opcStr = "arith";
        instCnt = (++instStat.arithCnt);
    }
    else if (opclass == DL_RISC_FMADD) {
        switch (opc) {
        case OPC_RISC_FMADD_S:
            opcStr = "fmadd.s";
            instCnt = (++instStat.fmaddCnt[0]);
            break;
        case OPC_RISC_FMADD_D:
            opcStr = "fmadd.d";
            instCnt = (++instStat.fmaddCnt[1]);
            break;
        case OPC_RISC_FMADD_H:
            opcStr = "fmadd.h";
            instCnt = (++instStat.fmaddCnt[2]);
            break;
        }
    }
    else if (opclass == DL_RISC_FMSUB) {
        switch (opc) {
        case OPC_RISC_FMSUB_S:
            opcStr = "fmsub.s";
            instCnt = (++instStat.fmsubCnt[0]);
            break;
        case OPC_RISC_FMSUB_D:
            opcStr = "fmsub.d";
            instCnt = (++instStat.fmsubCnt[1]);
            break;
        case OPC_RISC_FMSUB_H:
            opcStr = "fmsub.h";
            instCnt = (++instStat.fmsubCnt[2]);
            break;
        }

    }
    else if (opclass == DL_RISC_FNMADD) {
        switch (opc) {
        case OPC_RISC_FNMADD_S:
            opcStr = "fnmadd.s";
            instCnt = (++instStat.fnmaddCnt[0]);
            break;
        case OPC_RISC_FNMADD_D:
            opcStr = "fnmadd.d";
            instCnt = (++instStat.fnmaddCnt[1]);
            break;
        case OPC_RISC_FNMADD_H:
            opcStr = "fnmadd.h";
            instCnt = (++instStat.fnmaddCnt[2]);
            break;
        }
    }
    else if (opclass == DL_RISC_FNMSUB) {
        switch (opc) {
        case OPC_RISC_FNMSUB_S:
            opcStr = "fnmsub.s";
            instCnt = (++instStat.fnmsubCnt[0]);
            break;
        case OPC_RISC_FNMSUB_D:
            opcStr = "fnmsub.d";
            instCnt = (++instStat.fnmsubCnt[1]);
            break;
        case OPC_RISC_FNMSUB_H:
            opcStr = "fnmsub.h";
            instCnt = (++instStat.fnmsubCnt[2]);
            break;
        }
    }
    else if (opclass == DL_RISC_FP_ARITH) {
        opcStr = "fparith";
        instCnt = (++instStat.fparithCnt);
    }
    else if (opclass == DL_RISC_V_IVV) {
        opcStr = "varithi.vv";
        instCnt = (++instStat.varithiCnt[0]); // vv
    }
    else if (opclass == DL_RISC_V_IVX) {
        opcStr = "varithi.vx";
        instCnt = (++instStat.varithiCnt[1]); // vx
    }
    else if (opclass == DL_RISC_V_IVI) {
        opcStr = "varithi.vi";
        instCnt = (++instStat.varithiCnt[2]); // vi
    }
    else if (opclass == DL_RISC_V_MVV) {
        opcStr = "varithm.vv";
        instCnt = (++instStat.varithmCnt[0]); // vv
    }
    else if (opclass == DL_RISC_V_MVX) {
        opcStr = "varithm.vx";
        instCnt = (++instStat.varithmCnt[1]); // vx
    }
    else if (opclass == DL_RISC_V_FVV) {
        opcStr = "varithf.vv";
        instCnt = (++instStat.varithfCnt[0]); // vv
    }
    else if (opclass == DL_RISC_V_FVF) {
        opcStr = "varithf.vf";
        instCnt = (++instStat.varithfCnt[1]); // vf
    }
    fprintf(logfp, "[%lu] %s(=%04x)/%u: ", instStat.instCtr, opcStr, opc, instCnt);
#else // for binary format
    // uint32_t instCnt = 0;
    DLTraceCompactArith trace = { 0, };
    //trace.lower.opType = 1; // arith (traceV1, deprecated)
    trace.lower.opType = 2; // arith (traceV2)
    if (opclass == DL_RISC_ARITH_IMM) {
        // instCnt = (++instStat.arithImmCnt);
        trace.lower.dataType = 0;       // int
        trace.lower.operandSize = 2;    // 32
        ++instStat.arithImmCnt;
    }
    else if (opclass == DL_RISC_ARITH) {
        // instCnt = (++instStat.arithCnt);
        trace.lower.dataType = 0;       // int
        trace.lower.operandSize = 2;    // 32
        ++instStat.arithCnt;
    }
    else if (opclass == DL_RISC_FMADD) {
        trace.lower.dataType = 2; // fp (traceV2)
        switch (opc) {
        case OPC_RISC_FMADD_S:
            //opcStr = "fmadd.s";
            // instCnt = (++instStat.fmaddCnt[0]);
            trace.lower.operandSize = 2;    // FP32
            ++instStat.fmaddCnt[0];
            break;
        case OPC_RISC_FMADD_D:
            //opcStr = "fmadd.d";
            // instCnt = (++instStat.fmaddCnt[1]);
            trace.lower.operandSize = 3;    // FP64
            ++instStat.fmaddCnt[1];
            break;
        case OPC_RISC_FMADD_H:
            //opcStr = "fmadd.h";
            // instCnt = (++instStat.fmaddCnt[2]);
            trace.lower.operandSize = 1;    // FP16
            ++instStat.fmaddCnt[2];
            break;
        }
    }
    else if (opclass == DL_RISC_FMSUB) {
        trace.lower.dataType = 2; // fp (traceV2)
        switch (opc) {
        case OPC_RISC_FMSUB_S:
            //opcStr = "fmsub.s";
            // instCnt = (++instStat.fmsubCnt[0]);
            trace.lower.operandSize = 2;    // FP32
            ++instStat.fmsubCnt[0];
            break;
        case OPC_RISC_FMSUB_D:
            //opcStr = "fmsub.d";
            // instCnt = (++instStat.fmsubCnt[1]);
            trace.lower.operandSize = 3;    // FP64
            ++instStat.fmsubCnt[1];
            break;
        case OPC_RISC_FMSUB_H:
            //opcStr = "fmsub.h";
            // instCnt = (++instStat.fmsubCnt[2]);
            trace.lower.operandSize = 1;    // FP16
            ++instStat.fmsubCnt[2];
            break;
        }
    }
    else if (opclass == DL_RISC_FNMADD) {
        trace.lower.dataType = 2; // fp (traceV2)
        switch (opc) {
        case OPC_RISC_FNMADD_S:
            //opcStr = "fnmadd.s";
            // instCnt = (++instStat.fnmaddCnt[0]);
            trace.lower.operandSize = 2;    // FP32
            ++instStat.fnmaddCnt[0];
            break;
        case OPC_RISC_FNMADD_D:
            //opcStr = "fnmadd.d";
            // instCnt = (++instStat.fnmaddCnt[1]);
            trace.lower.operandSize = 3;    // FP64
            ++instStat.fnmaddCnt[1];
            break;
        case OPC_RISC_FNMADD_H:
            //opcStr = "fnmadd.h";
            // instCnt = (++instStat.fnmaddCnt[2]);
            trace.lower.operandSize = 1;    // FP16
            ++instStat.fnmaddCnt[2];
            break;
        }
    }
    else if (opclass == DL_RISC_FNMSUB) {
        trace.lower.dataType = 2; // fp (traceV2)
        switch (opc) {
        case OPC_RISC_FNMSUB_S:
            //opcStr = "fnmsub.s";
            // instCnt = (++instStat.fnmsubCnt[0]);
            trace.lower.operandSize = 2;    // FP32
            ++instStat.fnmsubCnt[0];
            break;
        case OPC_RISC_FNMSUB_D:
            // opcStr = "fnmsub.d";
            // instCnt = (++instStat.fnmsubCnt[1]);
            trace.lower.operandSize = 3;    // FP64
            ++instStat.fnmsubCnt[1];
            break;
        case OPC_RISC_FNMSUB_H:
            // opcStr = "fnmsub.h";
            // instCnt = (++instStat.fnmsubCnt[2]);
            trace.lower.operandSize = 1;    // FP16
            ++instStat.fnmsubCnt[2];
            break;
        }
    }
    else if (opclass == DL_RISC_FP_ARITH) {
        // opcStr = "fparith";
        // instCnt = (++instStat.fparithCnt);
        trace.lower.dataType = 2;       // fp (traceV2)
        trace.lower.operandSize = 2;    // FP32
        ++instStat.fparithCnt;
    }
    else if (opclass == DL_RISC_V_IVV) {
        // opcStr = "varithi.vv";
        // instCnt = (++instStat.varithiCnt[0]); // vv
        trace.lower.dataType = 3;       // vector (traceV2)
        trace.lower.operandSize = 2;    // 32?
        ++instStat.varithiCnt[0];
    }
    else if (opclass == DL_RISC_V_IVX) {
        // opcStr = "varithi.vx";
        // instCnt = (++instStat.varithiCnt[1]); // vx
        trace.lower.dataType = 3;       // vector
        trace.lower.operandSize = 2;
        ++instStat.varithiCnt[1];
    }
    else if (opclass == DL_RISC_V_IVI) {
        // opcStr = "varithi.vi";
        // instCnt = (++instStat.varithiCnt[2]); // vi
        trace.lower.dataType = 3;       // vector
        trace.lower.operandSize = 2;
        ++instStat.varithiCnt[2];
    }
    else if (opclass == DL_RISC_V_MVV) {
        // opcStr = "varithm.vv";
        // instCnt = (++instStat.varithmCnt[0]); // vv
        trace.lower.dataType = 3;       // vector
        trace.lower.operandSize = 2;
        ++instStat.varithmCnt[0];
    }
    else if (opclass == DL_RISC_V_MVX) {
        // opcStr = "varithm.vx";
        // instCnt = (++instStat.varithmCnt[1]); // vx
        trace.lower.dataType = 3;       // vector
        trace.lower.operandSize = 2;
        ++instStat.varithmCnt[1];
    }
    else if (opclass == DL_RISC_V_FVV) {
        // opcStr = "varithf.vv";
        // instCnt = (++instStat.varithfCnt[0]); // vv
        trace.lower.dataType = 3;       // vector
        trace.lower.operandSize = 2;
        ++instStat.varithfCnt[0];
    }
    else if (opclass == DL_RISC_V_FVF) {
        // opcStr = "varithf.vf";
        // instCnt = (++instStat.varithfCnt[1]); // vf
        trace.lower.dataType = 3;       // vector
        trace.lower.operandSize = 2;
        ++instStat.varithfCnt[1];
    }
    else { // unknown or custom
        trace.lower.opType = 3; // traceV2
        ++instStat.unknownCnt; // TODO: unknown과 custom 세분화
    }
    trace.instCtr = instStat.instCtr;
    fwrite(&trace.lower, sizeof(DLTraceLow), 1, logfp);
    fwrite(&trace.instCtr, sizeof(uint64_t), 1, logfp);
    //fprintf(logfp, "[%lu] %s(=%04x)/%u: ", instStat.instCtr, opcStr, opc, instCnt);
#endif
}

void dl_put_opc_vector(uint32_t opc, uint32_t width) {
    char *opcStr = "unknown";
    uint32_t instCnt = 0;
    //uint32_t nwidth = ((width & 0x3) <= 3) ? 8 << (width & 0x3) : 0;
    uint32_t mw = width & 0x03;
    uint32_t nwidth = 8 << mw;

    switch (opc) {
    // load
    case OPC_RISC_VL_US:  // unit-stride
        instCnt = (++instStat.vleCnt[mw]);
        opcStr = "vle";
        break;
    case OPC_RISC_VL_VS:  // vector-strided
        instCnt = (++instStat.vlseCnt[mw]);
        opcStr = "vlse";
        break;
    case OPC_RISC_VL_UVI: // unordered vector-indexed
    case OPC_RISC_VL_OVI: // ordered vector-indexed
        instCnt = (++instStat.vlxeiCnt[mw]);
        opcStr = "vlxei";
        break;
    // store
    case OPC_RISC_VS_US:  // unit-stride
        instCnt = (++instStat.vseCnt[mw]);
        opcStr = "vse";
        break;
    case OPC_RISC_VS_VS:  // vector-strided
        instCnt = (++instStat.vsseCnt[mw]);
        opcStr = "vsse";
        break;
    case OPC_RISC_VS_UVI: // unordered vector-indexed
    case OPC_RISC_VS_OVI: // ordered vector-indexed
        instCnt = (++instStat.vsxeiCnt[mw]);
        opcStr = "vsxei";
        break;
    }
    fprintf(logfp, "[%lu] %s%d(=%04x)/%u: ", instStat.instCtr, opcStr, nwidth, opc, instCnt);
}

uint8_t dl_put_opc_vector_bin(uint32_t opc, uint32_t width) {
    //uint32_t instCnt = 0;
    uint32_t mw = width & 0x03;
    //uint32_t nwidth = 8 << mw;
    uint8_t opclass = 0;
    DLTraceCompactMem trace = { 0, };
    //trace.lower.opType = 0;     // mem (traceV1, deprecated)
    trace.lower.dataType = 3;   // vector (traceV2)
    trace.lower.operandSize = mw;
    switch (opc) {
    // load
    case OPC_RISC_VL_US:  // unit-stride
        // opcStr = "vle";
        //instCnt = (++instStat.vleCnt[mw]);
        trace.lower.opType = 0; // load (traceV2)
        ++instStat.vleCnt[mw];
        opclass = DL_RISC_VL_US;
        break;
    case OPC_RISC_VL_VS:  // vector-strided
        // opcStr = "vlse";
        //instCnt = (++instStat.vlseCnt[mw]);
        trace.lower.opType = 0; // load
        ++instStat.vlseCnt[mw];
        opclass = DL_RISC_VL_VS;
        break;
    case OPC_RISC_VL_UVI: // unordered vector-indexed
    case OPC_RISC_VL_OVI: // ordered vector-indexed
        // opcStr = "vlxei";
        // instCnt = (++instStat.vlxeiCnt[mw]);
        trace.lower.opType = 0; // load
        ++instStat.vlxeiCnt[mw];
        opclass = DL_RISC_VL_UVI;
        break;
    // store
    case OPC_RISC_VS_US:  // unit-stride
        // opcStr = "vse";
        // instCnt = (++instStat.vseCnt[mw]);
        trace.lower.opType = 1; // store (traceV2)
        ++instStat.vseCnt[mw];
        opclass = DL_RISC_VS_US;
        break;
    case OPC_RISC_VS_VS:  // vector-strided
        // opcStr = "vsse";
        // instCnt = (++instStat.vsseCnt[mw]);
        trace.lower.opType = 1; // store
        opclass = DL_RISC_VS_VS;
        ++instStat.vsseCnt[mw];
        break;
    case OPC_RISC_VS_UVI: // unordered vector-indexed
    case OPC_RISC_VS_OVI: // ordered vector-indexed
        // opcStr = "vsxei";
        // instCnt = (++instStat.vsxeiCnt[mw]);
        trace.lower.opType = 1; // store
        opclass = DL_RISC_VS_UVI;
        ++instStat.vsxeiCnt[mw];
        break;
    default: // unknown
        trace.lower.opType = 3; // unknown/custom (traceV2)
        // trace.lower.dataType = 0b111;
        // trace.lower.operandSize = 0b1111;
        ++instStat.unknownCnt;
        opclass = DL_RISC_UNKNOWN;
        break;
    }
    //fprintf(logfp, "[%lu] %s%d(=%04x)/%u: ", instStat.instCtr, opcStr, nwidth, opc, instCnt);
    trace.instCtr = instStat.instCtr;
    fwrite(&trace.lower, sizeof(DLTraceLow), 1, logfp);
    fwrite(&trace.instCtr, sizeof(uint64_t), 1, logfp);
    return opclass;
}

int dl_get_custom_instcnt_idx(uint64_t opc) {
    uint32_t opcode = (uint32_t)opc & 0b1111111;
    uint32_t funct3 = ((uint32_t)opc >> 12) & 0b111;
    if (opcode == DL_OPCODE_CUSTOM0) {
        switch (funct3) {
        case 0:
            return DL_CUSTOM0_DRBEGIN;
        case 1:
            return DL_CUSTOM0_DREND;
        }
    }
    else if (opcode == DL_OPCODE_CUSTOM3) {
        switch (funct3) {
        case 0: // simprint
            return DL_CUSTOM3_SPRINGBOK_SIMPRINT;
        case 1: // xcount
            return DL_CUSTOM3_SPRINGBOK_XCOUNT;
        case 2: // hostreq
            return DL_CUSTOM3_SPRINGBOK_HOSTREQ;
        case 3: // finish
            return DL_CUSTOM3_SPRINGBOK_FINISH;
        }
    }
    return -1; // unknown
}

void dl_put_opc_custom(uint64_t opc) {
    int idx = dl_get_custom_instcnt_idx(opc);
    if (idx != -1) {
        ++instStat.customCnt[idx];
        fprintf(logfp, "[%lu] custom(=%04x)/%u: pc=%08x\n", instStat.instCtr, (uint32_t)opc, instStat.customCnt[idx], (uint32_t)env->pc);
    }
    else {
        ++instStat.unknownCnt;
        fprintf(logfp, "[%lu] unknown(=%04x)/%u: pc=%08x\n", instStat.instCtr, (uint32_t)opc, instStat.unknownCnt, (uint32_t)env->pc);
    }
}

void dl_put_opc_custom_bin(uint64_t opc) {
    /* TraceV2 custom instruction field
    opclass[1:0]: opcode
    0b00: custom-0
    0b01: custom-1
    0b10: custom-2
    0b11: custom-3

    opclass[2:4]: funct3
    opclass[5:7]: reserved
    --> 0b111: unknown

    uint8_t opclass = 0;
    opclass |= opcode;
    opclass |= (funct3 << 2);
    opclass |= (reserved << 5);
    */
    DLTraceCompactArith trace = { 0, };
    trace.lower.opType = 3; // custom/unknown

    uint8_t opclass = 0;
    int idx = dl_get_custom_instcnt_idx(opc);

    if (idx != -1) {
        uint8_t opcode = (uint8_t)opc & 0b1111111;
        uint8_t funct3 = (uint8_t)(opc >> 12) & 0b111;
        switch (opcode) {
        case DL_OPCODE_CUSTOM0:
            //printf("[%lu] custom-0 %s (funct3=%d) ", instStat.instCtr, DL_CUSTOM_INSTRUCTION_MNEMONIC[idx], funct3);
            opclass |= 0b00;
            break;
        case DL_OPCODE_CUSTOM1:
            //printf("[%lu] custom-1 %s (funct3=%d) ", instStat.instCtr, DL_CUSTOM_INSTRUCTION_MNEMONIC[idx], funct3);
            opclass |= 0b01;
            break;
        case DL_OPCODE_CUSTOM2:
            //printf("[%lu] custom-2 %s (funct3=%d) ", instStat.instCtr, DL_CUSTOM_INSTRUCTION_MNEMONIC[idx], funct3);
            opclass |= 0b10;
            break;
        case DL_OPCODE_CUSTOM3:
            //printf("[%lu] custom-3 %s (funct3=%d) ", instStat.instCtr, DL_CUSTOM_INSTRUCTION_MNEMONIC[idx], funct3);
            opclass |= 0b11;
            break;
        }
        opclass |= funct3 << 2;
        // reserved field: 일단 일반 커스텀 명령어는 0으로 채움
        ++instStat.customCnt[idx];
    }
    else { // unknown
        opclass |= 0b111 << 5;
        ++instStat.unknownCnt;
    }
    //printf("opclass=%02x\n", opclass);
    trace.instCtr = instStat.instCtr;
    trace.addr = (uint32_t)env->pc;
    trace.opclass = opclass;
    fwrite(&trace.lower, sizeof(DLTraceLow), 1, logfp);
    fwrite(&trace.instCtr, sizeof(uint64_t), 1, logfp);
    fwrite(&trace.addr, sizeof(uint32_t), 1, logfp);
    fwrite(&trace.opclass, sizeof(uint8_t), 1, logfp);
}

void dl_print_inst_stat(void) {
    uint32_t intLoadTotal = 0, intStoreTotal = 0;
    uint32_t floatLoadTotal = 0, floatStoreTotal = 0;
    uint32_t vectorLoadTotal = 0, vectorStoreTotal = 0;
    uint32_t fmaddTotal = 0, fmsubTotal = 0;
    uint32_t fnmaddTotal = 0, fnmsubTotal = 0;
    //uint32_t loadTotal = 0, storeTotal = 0;

#ifdef DL_TRACE_HUMAN_READABLE
    FILE *fpTarget = logfp;
#else
    FILE *fpTarget = stdout;
#endif    

    fprintf(fpTarget, "## Scala integer loads ##\n");
    fprintf(fpTarget, "ld:  %u\n", instStat.ldCnt);
    fprintf(fpTarget, "lw:  %u\n", instStat.lwCnt);
    fprintf(fpTarget, "lwu: %u\n", instStat.lwuCnt);
    fprintf(fpTarget, "lh:  %u\n", instStat.lhCnt);
    fprintf(fpTarget, "lhu: %u\n", instStat.lhuCnt);
    fprintf(fpTarget, "lb:  %u\n", instStat.lbCnt);
    fprintf(fpTarget, "lbu: %u\n\n", instStat.lbuCnt);
    intLoadTotal += instStat.ldCnt;
    intLoadTotal += instStat.lwCnt;
    intLoadTotal += instStat.lwuCnt;
    intLoadTotal += instStat.lhCnt;
    intLoadTotal += instStat.lhuCnt;
    intLoadTotal += instStat.lbCnt;
    intLoadTotal += instStat.lbuCnt;

    fprintf(fpTarget, "## Scala integer stores ##\n");
    fprintf(fpTarget, "sd: %u\n", instStat.sdCnt);
    fprintf(fpTarget, "sw: %u\n", instStat.swCnt);
    fprintf(fpTarget, "sh: %u\n", instStat.shCnt);
    fprintf(fpTarget, "sb: %u\n\n", instStat.sbCnt);
    intStoreTotal += instStat.sdCnt;
    intStoreTotal += instStat.swCnt;
    intStoreTotal += instStat.shCnt;
    intStoreTotal += instStat.sbCnt;

    fprintf(fpTarget, "## Scala FP loads ##\n");
    fprintf(fpTarget, "fld: %u\n", instStat.fldCnt);
    fprintf(fpTarget, "flw: %u\n", instStat.flwCnt);
    fprintf(fpTarget, "flh: %u\n\n", instStat.flhCnt);
    floatLoadTotal += instStat.fldCnt;
    floatLoadTotal += instStat.flwCnt;
    floatLoadTotal += instStat.flhCnt;

    fprintf(fpTarget, "## Scala FP stores ##\n");
    fprintf(fpTarget, "fsd: %u\n", instStat.fsdCnt);
    fprintf(fpTarget, "fsw: %u\n", instStat.fswCnt);
    fprintf(fpTarget, "fsh: %u\n\n", instStat.fshCnt);
    floatStoreTotal += instStat.fsdCnt;
    floatStoreTotal += instStat.fswCnt;
    floatStoreTotal += instStat.fshCnt;

    fprintf(fpTarget, "## Vector loads ##\n");
    int i;
    for (i = 0; i < 4; i++) {
        fprintf(fpTarget, "vle%d: %u\n", 8 << i, instStat.vleCnt[i]);
        fprintf(fpTarget, "vlse%d: %u\n", 8 << i, instStat.vlseCnt[i]);
        fprintf(fpTarget, "vlxei%d: %u\n\n", 8 << i, instStat.vlxeiCnt[i]);
        vectorLoadTotal += instStat.vleCnt[i];
        vectorLoadTotal += instStat.vlseCnt[i];
        vectorLoadTotal += instStat.vlxeiCnt[i];
    }

    fprintf(fpTarget, "## Vector stores ##\n");
    for (i = 0; i < 4; i++) {
        fprintf(fpTarget, "vse%d: %u\n", 8 << i, instStat.vseCnt[i]);
        fprintf(fpTarget, "vsse%d: %u\n", 8 << i, instStat.vsseCnt[i]);
        fprintf(fpTarget, "vsxei%d: %u\n\n", 8 << i, instStat.vsxeiCnt[i]);
        vectorLoadTotal += instStat.vseCnt[i];
        vectorLoadTotal += instStat.vsseCnt[i];
        vectorLoadTotal += instStat.vsxeiCnt[i];
    }

    const char *fpsuffix[] = { ".s", ".d", ".h" };
    fprintf(fpTarget, "## fmadd ##\n");
    for (i = 0; i < 3; i++) {
        fprintf(fpTarget, "fmadd%s: %u\n", fpsuffix[i], instStat.fmaddCnt[i]);
        fmaddTotal += instStat.fmaddCnt[i];
    }
    fprintf(fpTarget, "\n");

    fprintf(fpTarget, "## fmsub ##\n");
    for (i = 0; i < 3; i++) {
        fprintf(fpTarget, "fmsub%s: %u\n", fpsuffix[i], instStat.fmsubCnt[i]);
        fmaddTotal += instStat.fmsubCnt[i];
    }
    fprintf(fpTarget, "\n");

    fprintf(fpTarget, "## fnmadd ##\n");
    for (i = 0; i < 3; i++) {
        fprintf(fpTarget, "fnmadd%s: %u\n", fpsuffix[i], instStat.fnmaddCnt[i]);
        fmaddTotal += instStat.fnmaddCnt[i];
    }
    fprintf(fpTarget, "\n");

    fprintf(fpTarget, "## fnmsub ##\n");
    for (i = 0; i < 3; i++) {
        fprintf(fpTarget, "fnmsub%s: %u\n", fpsuffix[i], instStat.fnmsubCnt[i]);
        fmaddTotal += instStat.fnmsubCnt[i];
    }
    fprintf(fpTarget, "\n");

    const char *vsuffix[] = { ".vv", ".vx", ".vi" };
    uint32_t varithiTotal = 0;
    uint32_t varithmTotal = 0;
    uint32_t varithfTotal = 0;
    fprintf(fpTarget, "## Vector arithmetic ##\n");
    for (i = 0; i < 3; i++) {
        fprintf(fpTarget, "varithi%s: %u\n", vsuffix[i], instStat.varithiCnt[i]);
        varithiTotal += instStat.varithiCnt[i];
    }
    for (i = 0; i < 2; i++) {
        fprintf(fpTarget, "varithm%s: %u\n", vsuffix[i], instStat.varithmCnt[i]);
        varithmTotal += instStat.varithmCnt[i];
    }
    for (i = 0; i < 2; i++) {
        const char *suffix = vsuffix[i];
        if (i == 1)
            suffix = ".vf";
        fprintf(fpTarget, "varithf%s: %u\n", suffix, instStat.varithfCnt[i]);
        varithfTotal += instStat.varithfCnt[i];
    }
    fprintf(fpTarget, "\n");

    fprintf(fpTarget, "## custom instructions and unknown ##\n");
    for (i = 0; i < DL_CUSTOM_INSTRUCTION_COUNT; i++) {
        fprintf(fpTarget, "%s: %u\n", DL_CUSTOM_INSTRUCTION_MNEMONIC[i], instStat.customCnt[i]);
    }
    fprintf(fpTarget, "unknown: %u\n\n", instStat.unknownCnt);

    fprintf(fpTarget, "## Total instruction count ##\n");
    fprintf(fpTarget, "load: %u\n", intLoadTotal);
    fprintf(fpTarget, "store: %u\n", intStoreTotal);
    fprintf(fpTarget, "FP load: %u\n", floatLoadTotal);
    fprintf(fpTarget, "FP store: %u\n", floatStoreTotal);
    fprintf(fpTarget, "Vector load: %u\n", vectorLoadTotal);
    fprintf(fpTarget, "Vector store: %u\n", vectorStoreTotal);

    fprintf(fpTarget, "Total load: %u\n", intLoadTotal + floatLoadTotal + vectorLoadTotal);
    fprintf(fpTarget, "Total store: %u\n", intStoreTotal + floatStoreTotal + vectorStoreTotal);

    fprintf(fpTarget, "Total arith imm.: %u\n", instStat.arithImmCnt);
    fprintf(fpTarget, "Total arith: %u\n", instStat.arithCnt);

    fprintf(fpTarget, "FP arith: %u\n", instStat.fparithCnt);
    fprintf(fpTarget, "Total fmadd: %u\n", fmaddTotal);
    fprintf(fpTarget, "Total fmsub: %u\n", fmsubTotal);
    fprintf(fpTarget, "Total fnmadd: %u\n", fnmaddTotal);
    fprintf(fpTarget, "Total fnmsub: %u\n", fnmsubTotal);
    fprintf(fpTarget, "Total FP arith: %u\n", instStat.fparithCnt + fmaddTotal + fmsubTotal + fnmaddTotal + fnmsubTotal);

    fprintf(fpTarget, "Total vector arith (i): %u\n", varithiTotal);
    fprintf(fpTarget, "Total vector arith (m): %u\n", varithmTotal);
    fprintf(fpTarget, "Total vector arith (f): %u\n", varithfTotal);
    fprintf(fpTarget, "Total vector arith: %u\n", varithiTotal + varithmTotal + varithfTotal);

    fprintf(fpTarget, "Total instructions: %lu\n", instStat.instCtr);
}

void dl_close_log_fp(void) {
    if (logfp) {
        fclose(logfp);
        logfp = NULL;
        printf("dl_close_log_fp(): %s is closed\n", pathName);
    }
}