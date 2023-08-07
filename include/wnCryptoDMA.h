
/**************************************************************************/
/*                                                                        */
/* The content of this file or document is CONFIDENTIAL and PROPRIETRY    */
/* to  Adeptchips Services Pvt Ltd. It  is  subject  to  the terms of a   */
/* License Agreement  between Licensee and  Apeptchips Services Pvt Ltd.  */
/* restricting  among other things, the use, reproduction, distribution   */
/* and transfer. Each of the embodiments, including this information and  */
/* any derivative work shall retain this copyright notice                 */
/*                                                                        */
/* Copyright Adeptchips Services Pvt Ltd.                                 */
/* All rights reserved                                                    */
/*                                                                        */
/**************************************************************************/

#ifndef __CRYPTODMA_H_
#define __CRYPTODMA_H__

#include "wn5gNrPsDataTypes.h"
#include "wn5gNrPsErrTypes.h"
#include "wisig.h"

#define DMA_BASE 			((wnCryptoDMA *) CRYPTO_DMA_BASE)

//#define BUFF_LEN	0x51  	//81
#define BUFF_LEN	0x40	//64 bytes
//#define BUFF_LEN	0x80	//128 bytes
//#define BUFF_LEN	0x200	//512 bytes
//#define BUFF_LEN	0x400	//1K bytes
//#define BUFF_LEN	0x600	//1.5kb 
//#define BUFF_LEN	0x800	//2K bytes - not supported

#define LASTBD_ENB			0x1
#define LASTBD_DIS			0x0

#define ALGO_AES 			0b0000100
#define ALGO_TDES 			0b0000010
#define ALGO_DES 			0b0000001
#define ALGO_MD5			0b0001000
#define ALGO_SHA1			0b0010000
#define ALGO_SHA256			0b0100000
#define ALGO_HMAC			0b1000000

#define AES_SHA1			0b0010100
#define AES_SHA256			0b0100100
#define HMAC_SHA1			0b1010000
#define HMAC_SHA256		0b1100000

#define HMAC_SHA1_AES_RCBC  0b1010100

#define ENCRYPT				0x1
#define DECRYPT 			0x0

#define KEY_256				0b10
#define	KEY_192 			0b01
#define KEY_128				0b00
#define KEY_512				0b11

/* AES_ALGORITHM */
#define CRYPTOALGO_AES_GCM  0b1110
#define CRYPTOALGO_RCTR     0b1101
#define CRYPTOALGO_RCBC_MAC 0b1100
#define CRYPTOALGO_ROFB     0b1011
#define CRYPTOALGO_RCFB     0b1010
#define CRYPTOALGO_RCBC     0b1001
#define CRYPTOALGO_RECB     0b1000

/* Triple-DES_ALGORITHM */
#define CRYPTOALGO_TOFB     0b0111
#define CRYPTOALGO_TCFB     0b0110
#define CRYPTOALGO_TCBC     0b0101
#define CRYPTOALGO_TECB     0b0100

/* DES_ALGORIHTM */
#define CRYPTOALGO_OFB      0b0011
#define CRYPTOALGO_CFB      0b0010
#define CRYPTOALGO_CBC      0b0001
#define CRYPTOALGO_ECB      0b0000

#define MULTITASK_FREEPASS		0b000
#define MULTITASK_PIPEPASS		0b101
#define MULTITASK_PARALLELPASS	0b111

#define KEYSIZE_16	16
#define KEYSIZE_24	24
#define KEYSIZE_32	32


/* Cryto_DMA structure */
typedef struct {
	wnUInt32 VERSION;     				/* Offset : 0x00 CRDMA_Version register*/
	wnUInt32 CTRL;						/* Offset : 0x04 CRDMA_Control register*/
	wnUInt32 CURR_BD_ADDR;				/* Offset : 0x08 CRDMA_Curr_Bd_addrss register*/
	wnUInt32 BDP_ADDR;   				/* Offset : 0x0C CRDMA_BDP_Address register*/
	wnUInt32 STATUS;					/* Offset : 0x10 CRDMA_Status register*/
	wnUInt32 INT_SRC;  					/* Offset : 0x14 CRDMA_Intrpt_Src register*/
	wnUInt32 INT_EN;					/* Offset : 0x18 CRDMA_Intrpt_Enable register*/
	wnUInt32 POLL_CTRL;					/* Offset : 0x1c CRDMA_Poll_ctrl register*/
	wnUInt32 HEADER_LEN;				/* Offset : 0x20 CRDMA_Header_len register*/
	wnUInt32 TRAILER_LEN;				/* Offset : 0x24 CRDMA_trailer_len register*/	
	wnUInt32 DTX_STATUS_ADDR;			/* Offset : 0x28 CRDMA_DTx_Status_Address register*/
	wnUInt32 DRX_STATUS_ADDR;			/* Offset : 0x2c CRDMA_DRx_Status_Address register*/
	wnUInt32 CEN_CSR_CRYPTO_ALGO_ADDR;	/* Offset : 0x30 CRDMA_CEN_CSR_Crypto_Algorthm_Address register*/
	wnUInt32 CEN_CSR_HASH_ALGO_ADDR;	/* Offset : 0x34 CRDMA_CEN_CSR_Hash_Algorthm_Address register*/
	wnUInt32 CEN_CSR_TXD_ALGO_ADDR;		/* Offset : 0x38 CRDMA_CEN_CSR_TxD_Algorthm_Address register*/
	wnUInt32 CEN_CSR_MSG_LEN0_ADDR;		/* Offset : 0x3c CRDMA_CEN_CSR_Msg_Len0_Address register*/
	wnUInt32 CEN_CSR_MSG_LEN1_ADDR;		/* Offset : 0x40 CRDMA_CEN_CSR_Msg_Len1_Address register*/
	wnUInt32 CEN_CSR_OFFSET_LEN0_ADDR;	/* Offset : 0x44 CRDMA_CEN_CSR_Offset_Len0_Address register*/
	wnUInt32 CEN_CSR_GCMSM_CSTATE_ADDR;	/* Offset : 0x48 CRDMA_CEN_CSR_GCMSM_Cstate_Address register*/
	wnUInt32 CEN_CSR_MULTITASK_ADDR;	/* Offset : 0x4c CRDMA_CEN_CSR_Multitask_Address register*/
	wnUInt32 SADMEM_CSR_SACTRL;			/* Offset : 0x50 CRDMA_Sadmemory_CSR_SACTR register*/
	wnUInt32 CTRLSM_CSR_CSTATE;			/* Offset : 0x54 CRDMA_CTRLSM_CSR_CSTATE register*/
	wnUInt32 DTX_CSR_LBUF_BYTECNT;		/* Offset : 0x58 CRDMA_DTX_CSR_LBUF_ByteCount register*/
	wnUInt32 DRX_CSR_LBUF_BYTECNT;		/* Offset : 0x5c CRDMA_DRX_CSR_LBUF_ByteCount register*/

} wnCryptoDMA;

/*Buffer descriptor control structure*/
typedef struct {
	wnUInt32 BUFLEN : 16;
    wnUInt32 CBD_INT_EN : 1;
    wnUInt32 PKT_INT_EN : 1;
    wnUInt32 LIFM : 1;
    wnUInt32 LAST_BD : 1;
    wnUInt32 : 1;
    wnUInt32 SAD_MEM_EN : 1;
    wnUInt32 SA_FETCH_EN : 1;
    wnUInt32 : 4;
    wnUInt32 CRY_MODE : 3;
    wnUInt32 : 1;
    volatile wnUInt32 DESC_EN : 1;
} wnBDcontrol;

/*Buffer Descriptor structure*/
typedef struct {
	wnBDcontrol BD_CTRL;  		/* Buffer desc control */
	wnUInt32 SA_ADDR;			/* Security association data */
	wnUInt32 SRC_ADDR;  		/* Source address */
	wnUInt32 DST_ADDR;  		/* Destination address */
	wnUInt32 NXT_PTR;			/* Next buffer desc pointer */
	wnUInt32 UPDATE_PTR;			/* update pointer */
	wnUInt32 MSG_LEN;			/* Hash algorithm msg len */
	wnUInt32 ENC_OFF;			/* encryption off */
} wnBufferDesc;  

/*Security association control */
typedef struct {
	wnUInt32 CRYPTOALGO : 4;
    wnUInt32 MULTITASK : 3;
    wnUInt32 KEYSIZE : 2;
    wnUInt32 ENCTYPE : 1;
    wnUInt32 ALGO : 9;
    wnUInt32 : 1;
    wnUInt32 FLAGS : 1;
    wnUInt32 FB : 1;
    wnUInt32 LOADIV : 1;
    wnUInt32 LNC : 1;
    wnUInt32 IRFLAG : 1;
    wnUInt32 ICVONLY : 1;
    wnUInt32 OR_EN : 1;
    wnUInt32 NO_RX : 1;
    wnUInt32 : 1;
    wnUInt32 VERIFY : 1;
    wnUInt32 : 2;
} wnSAcontrol;

/*Security Association */
typedef struct securityAssociation {
    wnSAcontrol SA_CTRL;			/* Security association control */
    wnUInt32 SA_AUTHKEY[8];			/* Authentication Key */
    wnUInt32 SA_ENCKEY[8];			/* Encryption key */
    wnUInt32 SA_AUTHIV[8];			/* Authentication Init Vec */
    wnUInt32 SA_ENCIV[4];			/* Encryption Init Vec */
} wnSecurityAssociation;


/* Crypto DMA Register description */
#define CRDMA_CTRL_CSR_DMA_EN_MSK   				(0X01 << 0)      
#define CRDMA_CTRL_CSR_BDP_POLL_CTRL_EN_MSK			(0X01 << 1)
#define CRDMA_CTRL_CSR_BDP_CH_STRT_MSK  			(0X01 << 2)
#define CRDMA_CTRL_CSF_CONFIG_UPDATE_MSK			(0X01 << 4)
#define CRDMA_CTRL_CSR_SWP_EN_MSK					(0X01 << 5)
#define CRDMA_CTRL_SW_RESET_MSK						(0X01 << 6)

#define CRDMA_IPSEC_INT_EN_MSK						(0X01 << 0)
#define CRDMA_CSR_CBD_INT_EN_MSK					(0X01 << 1)
#define CRDMA_CSR_PKT_INT_EN_MSK					(0X01 << 2)
#define CRDMA_ACCESS_RESP_ERROR_INT_EN_MSK			(0X01 << 3)  

#define CRDMA_ACCESS_RESP_ERROR_INT_SRC				(0X01 << 3)  
#define CRDMA_CSR_PKT_INT_SRC						(0X01 << 2) 
#define CRDMA_CSR_CBD_INT_SRC						(0X01 << 1)
#define CRDMA_IPSEC_INT_SRC							(0X01 << 0)


wnVoid *wnMemset(wnVoid *dst, wnInt32 c, wnInt32 n);
wnVoid *wnMemcpy(wnVoid *dest, wnVoid *src,  wnUInt32 n);

/* CryptoDMA test application functions*/
wnInt32 wnEncryptDecrypt(wnUInt32 algo, wnUInt32 cryptoAlgo, wnUInt32 *key, wnUInt32 *IV);
wnInt32 wnAuthentication(wnUInt32 algo, wnUInt32 *key);
wnInt32 print_check( wnUInt32 start, wnUInt32 limit, wnChar *Src_buff, wnChar *Dest_buff);

/* CryptoDMA Buffer descriptor config functions */
wnVoid wnBufDescConfig(wnBufferDesc *BD, wnSecurityAssociation *SA, wnBufferDesc *BD_next, wnUInt32 lastbd, wnUInt32 buflen,
						wnChar *Src_buff, wnChar *Dest_buff);
						
/* CryptoDMA Security Association config function */
wnVoid wnSAConfig(wnSecurityAssociation *SA, wnUInt32 Algo, wnUInt8 CrypAlgo, wnUInt8 type, wnUInt8 keysize, wnUInt8 IVlen,
						wnUInt8 Multitask, wnUInt32 *key, wnUInt32 *IV);

/* CryptoDMA API functions */
wnVoid wnCrDMAInit(wnBufferDesc *BD, wnCryptoDMA *dma);
wnVoid wnCrDMAIRQEnable (wnCryptoDMA *dma);
wnVoid wnCrDMAIRQDisable (wnCryptoDMA *dma);
wnVoid wnCrDMAClrIRQ ( wnCryptoDMA *dma);
wnUInt32 wnCrDMAStatus (wnCryptoDMA *dma );

#endif

