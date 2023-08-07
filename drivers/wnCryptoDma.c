
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

#include "wnCryptoDMA.h"

/* ------------------------------------
Func	wnBufDesc_Config()
@Desc 	This function configures the Buffer Descriptor structure with required settings
		(which is stored in RAM)
@param 	BD: buffer descriptor address(stored in RAM) 
		SA: Security Association structure address(stored in RAM)
		BD_next: next buffer Descriptor address
		Src_buff, Dest_buff : source and destination buffer address
		lastbd: enables/disables last bd 
@return 
		void
-------------------------------------- */

wnVoid wnBufDescConfig(wnBufferDesc *BD, wnSecurityAssociation *SA, wnBufferDesc *BD_next, 
				wnUInt32 lastbd, wnUInt32 buflen, wnChar *Src_buff, wnChar *Dest_buff)
{	
	BD->BD_CTRL.BUFLEN = buflen;
	// checks for the odd bytes ( if so pad zero)
	wnUInt32 pad = buflen % 16;
	if (pad != 0)
	BD->BD_CTRL.BUFLEN += (16 - pad);
	
	BD->BD_CTRL.CBD_INT_EN = 1;
	BD->BD_CTRL.PKT_INT_EN = 1;
	BD->BD_CTRL.SA_FETCH_EN = 1;
	BD->BD_CTRL.LAST_BD = lastbd;
	BD->BD_CTRL.LIFM = 1; 
	
	BD->BD_CTRL.DESC_EN = 1;
	
	BD->SA_ADDR  = (wnUInt32)SA;
	BD->SRC_ADDR = (wnUInt32)Src_buff;	

	BD->NXT_PTR  = (wnUInt32)BD_next;
	
	if(SA->SA_CTRL.CRYPTOALGO == CRYPTOALGO_AES_GCM ) {
		BD->DST_ADDR = (wnUInt32)Dest_buff;
		BD->UPDATE_PTR = (wnUInt32)(Dest_buff + BD->BD_CTRL.BUFLEN);
	}

	else if((SA->SA_CTRL.ALGO & (0b1110000)) == SA->SA_CTRL.ALGO )
		BD->UPDATE_PTR = (wnUInt32) Dest_buff;
	
	else if((SA->SA_CTRL.ALGO & (0b1110111)) == SA->SA_CTRL.ALGO) {
		BD->UPDATE_PTR = (wnUInt32) Dest_buff +  BD->BD_CTRL.BUFLEN;
		BD->DST_ADDR = (wnUInt32)Dest_buff;		
	}
	else
		BD->DST_ADDR = (wnUInt32)Dest_buff;	
		
	BD->MSG_LEN = BD->BD_CTRL.BUFLEN;// Actual message size

	BD->ENC_OFF = 0x00000000; 
}

/* ------------------------------------
Func	wnSA_Config()
@Desc 	This function configures the Security Association structure with required settings
		(which is stored in RAM)
@param 
		SA: Security Association structure address(stored in RAM)
@return 
		void
-------------------------------------- */

wnVoid wnSAConfig(wnSecurityAssociation *SA, wnUInt32 Algo, wnUInt8 CrypAlgo, wnUInt8 type,
				wnUInt8 keysize, wnUInt8 IVlen, wnUInt8 Multitask, wnUInt32 *key, wnUInt32 *IV)
{
	wnUInt32 *dst;
	
	SA->SA_CTRL.ALGO    = Algo;	
	SA->SA_CTRL.ENCTYPE = type;
	
	SA->SA_CTRL.ICVONLY = 0; 
	SA->SA_CTRL.FB      = 1;
	SA->SA_CTRL.LNC     = 1;
	SA->SA_CTRL.LOADIV  = 1;
	SA->SA_CTRL.MULTITASK = Multitask;

	SA->SA_CTRL.CRYPTOALGO = CrypAlgo;
		
	switch(keysize){
		case 16:
			SA->SA_CTRL.KEYSIZE = KEY_128;
			break;
		case 18:
		case 24:
			SA->SA_CTRL.KEYSIZE = KEY_192;
			break;
		case 32:
			SA->SA_CTRL.KEYSIZE = KEY_256;
			break;
	}
	
	if(Algo & 0b0000111) {
		dst = (wnUInt32 *)(SA->SA_ENCKEY + ( sizeof(SA->SA_ENCKEY)/sizeof(wnUInt32) - keysize/sizeof(wnUInt32)));
		wnMemcpy(dst, key, keysize);
	
		if(IV) {
			dst = (wnUInt32 *)(SA->SA_ENCIV + ( (sizeof(SA->SA_ENCIV)/sizeof(wnUInt32)) - (16/sizeof(wnUInt32))));
			wnMemcpy(dst, IV, IVlen);
		}
	}
	
	if (Algo & ALGO_HMAC) { // for hashing
		dst = (wnUInt32 *)(SA->SA_AUTHKEY);
		wnMemcpy(dst, key, keysize);
	
		if(IV) {
			dst = (wnUInt32 *)(SA->SA_AUTHIV);
			wnMemcpy(dst, IV, IVlen);
		}
	}
}

/*------------------------------------------------------------------
Func	wnCrDMAInit()
@Desc   This function Initialises the CryptoDMA
@param  Buffer Descriptor address, CryptoDMA baseaddress 
@return
		void
--------------------------------------------------------------------*/

wnVoid wnCrDMAInit(wnBufferDesc *Buff_des, wnCryptoDMA *dma)
{
	dma->CTRL = CRDMA_CTRL_SW_RESET_MSK; //software reset
	
	while(dma->CTRL); // clears the software reset for next cycle.
	
	dma->BDP_ADDR = (wnUInt32)Buff_des;  // fetching Buffer descriptor address to BDP_ADDR reg
	
	wnCrDMAIRQEnable(dma); //Enable Interrupt
	
	dma->HEADER_LEN = 0x8; //Adding header length 
	dma->TRAILER_LEN = 0x8; //Adding tariler length
	
	/* Enabling the DMA and BDP( Buffer Descriptor Processor) with poll for descriptor valid bit */
	dma->CTRL = CRDMA_CTRL_CSR_DMA_EN_MSK | CRDMA_CTRL_CSR_BDP_CH_STRT_MSK | 
				CRDMA_CTRL_CSR_BDP_POLL_CTRL_EN_MSK;  
}

/*------------------------------------------------------------------
Func	wnCrDMAIRQEnable()
@Desc   This function Enables the interrupt
@param  CryptoDMA baseaddress  
@return
		void
--------------------------------------------------------------------*/

wnVoid wnCrDMAIRQEnable(wnCryptoDMA *dma)
{
	
	dma->INT_EN = CRDMA_IPSEC_INT_EN_MSK | CRDMA_CSR_CBD_INT_EN_MSK | 
				  CRDMA_CSR_PKT_INT_EN_MSK;
					
}

/*------------------------------------------------------------------
Func	wnCrDMAClrIRQ()
@Desc   This function clears the interrupt
@param  CryptoDMA baseaddress  
@return
		void
--------------------------------------------------------------------*/

wnVoid wnCrDMAClrIRQ(wnCryptoDMA *dma)
{
	dma->INT_SRC = CRDMA_IPSEC_INT_SRC | CRDMA_CSR_CBD_INT_SRC | CRDMA_CSR_PKT_INT_SRC;
	
}

/*------------------------------------------------------------------
Func	wnCrDMAIRQEnable()
@Desc   This function Disables the interrupt
@param  CryptoDMA baseaddress  
@return
		void
--------------------------------------------------------------------*/

wnVoid wnCrDMAIRQDisable(wnCryptoDMA *dma)
{
	
	dma->INT_EN &= ~(CRDMA_IPSEC_INT_EN_MSK | CRDMA_CSR_CBD_INT_EN_MSK | 
					CRDMA_CSR_PKT_INT_EN_MSK);
					
}

/*------------------------------------------------------------------
Func	wnCrDMAStatus()
@Desc   This function gets the crytoDMA status
@param  CryptoDMA baseaddress  
@return
		value : status value
--------------------------------------------------------------------*/

wnUInt32 wnCrDMAStatus(wnCryptoDMA *dma)
{
	wnUInt32 value;
	value = dma->STATUS;
	
	return value;
}

wnVoid *wnMemset(wnVoid *dst, wnInt32 c, wnInt32 n)
{
   wnUChar*p = dst;
    while (n--){
        *p++ = (wnUChar)c;
    }
    return dst;
}

wnVoid *wnMemcpy(wnVoid *dest, wnVoid *src,  wnUInt32 n)
{
    wnInt32 i;
   // Typecast src and dest addresses to (char *)
    wnChar *csrc = (wnChar *)src;
    wnChar *cdest = (wnChar *)dest;
  
   // Copy contents of src[] to dest[]
    for (  i=0; i<n; i++){
        cdest[i] = csrc[i];
	}
	cdest[i] = '\0';
   
    return cdest;
}