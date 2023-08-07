#ifndef __CRDMA_H__
#define __CRDMA_H__


#define CRDMA_VERSION                   0x00000000
#define CRDMA_CTRL                      0x00000004
#define CRDMA_CURR_BD_ADDR              0x00000008
#define CRDMA_BDP_ADDR                  0x0000000C
#define CRDMA_STATUS                    0x00000010
#define CRDMA_INT_SRC                   0x00000014
#define CRDMA_INT_EN                    0x00000018
#define CRDMA_POLL_CTRL                 0x0000001C
#define CRDMA_HEADER_LEN                0x00000020
#define CRDMA_TRAILER_LEN               0x00000024
#define CRDMA_DTX_STATUS_ADDR           0x00000028
#define CRDMA_DRX_STATUS_ADDR           0x0000002C
#define CRDMA_CEN_CSR_CRYPTO_ALGO_ADDR  0x00000030
#define CRDMA_CEN_CSR_HASH_ALGO_ADDR    0x00000034
#define CRDMA_CEN_CSR_TXD_ALGO_ADDR     0x00000038
#define CRDMA_CEN_CSR_MSG_LEN0_ADDR     0x0000003C
#define CRDMA_CEN_CSR_MSG_LEN1_ADDR     0x00000040
#define CRDMA_CEN_CSR_OFFSET_LEN0_ADDR  0x00000044
#define CRDMA_CEN_CSR_GCMSM_CSTATE_ADDR 0x00000048
#define CRDMA_CEN_CSR_MULTITASK_ADDR    0x0000004C
#define CRDMA_SADMEM_CSR_SACTRL         0x00000050
#define CRDMA_CTRLSM_CSR_CSTATE         0x00000054
#define CRDMA_DTX_CSR_LBUF_BYTECNT      0x00000058
#define CRDMA_DRX_CSR_LBUF_BYTECNT      0x0000005C






#endif /* __CRDMA_H__ */