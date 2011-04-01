#include <linux/types.h>
#include <linux/module.h>	/* Modules                        */
#include <linux/init.h>		/* Initdata                       */
#include <linux/timer.h>		/* Initdata                       */

#include "radio-tea5766.h"
#include "radio-rds-tea5766.h"

static _trdsbasic_t rdsbasic;
static int B_Flag;
//static TAFlists_t AFlist;
//static int NrOfAFlists;
//static int B_Method;
//static TEONlists EON;
//static int NrOfEONlists;

//                             1         2         3         4         5         6
//                             1234567890123456789012345678901234567890123456789012345678901234
//static const char RTclear[] = "                                                                ";
//static const char PTYclear[] = "        ";
//static const char TextClear[] = "                      "; 


static int BlockType;
static int Error;
static int InSync;
static int InReset;
static int InOverflow;

static unsigned char Group;		// RDS group identifier
//static char Text1[32+2],Text2[32+2];	// radio text
static unsigned char DataAddress;	// RDS data address
//static int TextFlag, OlTextFlag;	// text message flag A / B
//static int PTYNflag, OlPTYNflag;	// pty name message flag A / B
//static long MJD;			// modified julian day info from b4/c4 block
//static int RDStime_Flag;		// time data from b4 block
//static int PI_ON;			// eon pi code from d14 block
//static int TP_ON;			// eon tp flag from b14 block
//static int TA_ON;			// eon ta flag from b14 block
//static int EONselector;			// eon pointer from b14 block
//static int EONdata;			// eon data from c14 block
//static int EONarmed;
static int A0Groups;
//static TAFlist_t ChkList;
static int psn_char_pair_check = 0;

static struct timer_list psn_timeout_timer;
static unsigned int group_is_valid_for_psn = 0;
#define PSN_TIMEOUT	(60*HZ/1000) /*60ms*/
#define PSN_PAIR1	0x1
#define PSN_PAIR2	0x2
#define PSN_PAIR3	0x4
#define PSN_PAIR4	0x8
#define PSN_ALLPAIRS	(PSN_PAIR4|PSN_PAIR3|PSN_PAIR2|PSN_PAIR1)

/*
timeout use to be sure that the 2 characters received for the PSN are on the
right position a group is 4 blocks wide (A,B,C and D). A = PI code, B set the
position of the 2 characters aof the PSN in block D.  a group transmission
takes about 88ms, so when the B block is received, the D block shouold have
been received (88/4 -> 22ms * 3 = 66ms) 66ms later. To avoid problems we can
fix a timeout = 60ms to decide that position set in block B and characters
received in block D are not synchronized.
*/
static void rds_group_psn_timeout_handler(unsigned long data) {
// printk("rds_group_psn_timeout_handler()\n"); 
	group_is_valid_for_psn = 0;
	//since this flag is only an integer, we don't need any mutex...
}

static void rds_group_init_psn_timer(void)
{
	init_timer(&psn_timeout_timer);
	psn_timeout_timer.function = rds_group_psn_timeout_handler;
	psn_timeout_timer.data = 0;
}

static void rds_group_start_psn_timer(void)
{
	mod_timer(&psn_timeout_timer, (jiffies + PSN_TIMEOUT));
	group_is_valid_for_psn = 1;
}

static void rds_group_stop_psn_timer(void)
{
	del_timer(&(psn_timeout_timer));
	group_is_valid_for_psn = 0;
}

#if 0
static void ClearRadioText(void) {
	memcpy(Text1,TextClear,sizeof(TextClear));
	memcpy(Text2,TextClear,sizeof(TextClear));
	memcpy(rdsbasic.rt,RTclear,sizeof(RTclear));
//{clearradiotext}
}

static void ClearPTYname (void) {
// Procedure ClearPTYname;
// Begin
//   rdsbasic.PTYN := PTYclear;
// End;  {clearptyname}

	memcpy(rdsbasic.ptyn,PTYclear,sizeof(PTYclear));

}

static void ClearAFlists(void) {

	int i;

	memset(&AFlist,0,sizeof(TAFlist_t)*LIST_SIZE);
	memset(&ChkList,0,sizeof(TAFlist_t)*LIST_SIZE);
	NrOfAFlists= 0;
	B_Method    = FALSE;
	EONarmed    = FALSE;
	NrOfEONlists = 0;

	memset(EON,0,sizeof(TEONlists));
	for (i=0;i<LIST_SIZE;i++) {
		memcpy(EON[i].PINcode,PTYclear,sizeof(PTYclear));
		EON[i].Linking = 0xFFFF;
	}
}
#endif
void RDS_ClearData(void) 
{
	
	//rdsbasic init
	memset(&rdsbasic,0,sizeof(_trdsbasic_t));
	//program service name
	memset(rdsbasic.psn,0,sizeof(string8));
	//clear PTY name
//	memcpy(rdsbasic.ptyn,PTYclear,sizeof(PTYclear));
	//rds date
//	memcpy(rdsbasic.rdsdate,PTYclear,sizeof(PTYclear));
	//rds time
//	memcpy(rdsbasic.rdstime,PTYclear,sizeof(PTYclear));
	//enhance other network
//	rdsbasic.ownlink = 0xffff;

	//other init
//	TextFlag = FALSE;
//	PTYNflag = FALSE;
	/* clear rdsbasic.rt, Text1, Text2 */
//	ClearRadioText();
//	ClearPTYname();
	DataAddress = 0;
	A0Groups = 0;	
	psn_char_pair_check = 0;
	
	return;
	
}

void RDS_init(void)
{
	rds_group_init_psn_timer();
	RDS_ClearData();
}

void RDS_exit(void)
{
	rds_group_stop_psn_timer();
}
#if 0
static void UpdateDecoderInfo(void) {
	switch( (DataAddress & 0x03) ) {
		case 0 :
			if ((DataAddress & 0x04) == 0x04)
				rdsbasic.di |= 0x08;
			else
				rdsbasic.di &= ~(0x08);
		break;
		case 1 :
			if ((DataAddress & 0x04) == 0x04)
				rdsbasic.di |= 0x04;
			else
				rdsbasic.di &= ~(0x04);
		break;
		case 2 :
			if ((DataAddress & 0x04) == 0x04)
				rdsbasic.di |= 0x02;
			else
				rdsbasic.di &= ~(0x02);
		case 3 :
			if ((DataAddress & 0x04) == 0x04)
				rdsbasic.di |= 0x01;
			else
				rdsbasic.di &= ~(0x01);
		break;
	}
//{updatedecoderinfo}
}
#endif

static void ProgramServiceName(int HiByte, int LoByte)
{
	//CHR : Chr wandelt einen Integerwert in das korrespondierende ASCII-Zeichen um.
// 	rdsbasic.PSN[(DataAddress AND $03) SHL 1 + 1] := CHR(HiByte);
// 	rdsbasic.PSN[(DataAddress AND $03) SHL 1 + 2] := CHR(LoByte);
	int index = 0;

	if(group_is_valid_for_psn){
		switch((DataAddress % 4)){
			case 0:
				index = 0;
				psn_char_pair_check |= PSN_PAIR1;
				break;
			case 1:
				index = 2;
				psn_char_pair_check |= PSN_PAIR2;
				break;
			case 2:
				index = 4;
				psn_char_pair_check |= PSN_PAIR3;
				break;
			case 3:
				index = 6;
				psn_char_pair_check |= PSN_PAIR4;
				break;
			default:
				break;
		}	
		rdsbasic.psn[index ] = (char)(HiByte);
		rdsbasic.psn[index + 1] = (char)(LoByte);
		if(psn_char_pair_check == PSN_ALLPAIRS){
			printk(KERN_INFO "ProgramServiceName ,DataAddress=%d ,%c%c,pi=%d name = %s \n",
			DataAddress,(char)(HiByte),(char)(LoByte),rdsbasic.pi,rdsbasic.psn);
			rdsbasic.psn_ok = 1;
		}
		group_is_valid_for_psn = 0;
	}
} 

#if 0
static void ProgramTypeName(unsigned char Offset,int HiByte, int LoByte)
{
// 	rdsbasic.PTYN[(DataAddress AND $01) * 4 + 1 + (Offset * 2)] := CHR(HiByte);
//	rdsbasic.PTYN[(DataAddress AND $01) * 4 + 2 + (Offset * 2)] := CHR(LoByte);
	rdsbasic.ptyn[((DataAddress & 0x01) *4 ) + (Offset * 2)] = (char)(HiByte);
	rdsbasic.ptyn[((DataAddress & 0x01) *4 ) + 1 + (Offset * 2)] = (char)(LoByte);
}
 
static void FillRadioText(unsigned char Offset,int HiByte, int LoByte)
{
	printk(KERN_INFO "FillRadioText , DataAddress=%d ,offset=%d txt:%c%c\n",DataAddress,Offset,(char)(HiByte),(char)(LoByte));
	if((DataAddress & 0x08) == 0x08){
		if(Offset == 1){
// 			Text2[(DataAddress AND $07) SHL 2 + 1]  := CHR(HiByte);
// 			Text2[(DataAddress AND $07) SHL 2 + 2]  := CHR(LoByte);
			Text2[((DataAddress & 0x07) << 2) ] = (char)(HiByte);
			Text2[((DataAddress & 0x07) << 2) + 1] = (char)(LoByte);
		} else {
// 			Text2[(DataAddress AND $07) SHL 2 + 3]  := CHR(HiByte);
// 			Text2[(DataAddress AND $07) SHL 2 + 4]  := CHR(LoByte);
			Text2[((DataAddress & 0x07) << 2) + 2] = (char)(HiByte);
			Text2[((DataAddress & 0x07) << 2) + 3] = (char)(LoByte);
		}
	}else {
		if(Offset == 1){
// 			Text1[(DataAddress AND $07) SHL 2 + 1]  := CHR(HiByte);
// 			Text1[(DataAddress AND $07) SHL 2 + 2]  := CHR(LoByte);
			Text1[((DataAddress & 0x07) << 2) ] = (char)(HiByte);
			Text1[((DataAddress & 0x07) << 2) + 1] = (char)(LoByte);
		} else {
// 			Text1[(DataAddress AND $07) SHL 2 + 3]  := CHR(HiByte);
// 			Text1[(DataAddress AND $07) SHL 2 + 4]  := CHR(LoByte);
			Text1[((DataAddress & 0x07) << 2) + 2] = (char)(HiByte);
			Text1[((DataAddress & 0x07) << 2) + 3] = (char)(LoByte);
		}
	}
// 	rdsbasic.RT := Text1 + Text2;
	sprintf(rdsbasic.rt,"%s%s",Text1,Text2);
//rdsbasic.RT := 'test123';  
}	
  /*
Procedure MakeTime;
Var  Dummy,Offset : Integer;
     Tempo        : ShortString;
     RDS_H,RDS_M  : Integer;
     Begin              { maketime }
     With rdsbasic do
		     begin
		     If RDStime_Flag then
		     RDS_H := (HiByte AND $F0) SHR 4 + $10
		     else
		     RDS_H := (HiByte AND $F0) SHR 4;
     RDS_M  := (HiByte AND $0F) SHL 2 +
		     (LoByte AND $C0) SHR 6;
     OffSet :=  LoByte AND $1F;
     If (Offset > 0) then
		     begin
		     Dummy := RDS_H * 60 + RDS_M;
     If ((Offset * 30) > Dummy) then
		     begin
		     DEC(MJD, 1);
     MakeDate;
     Dummy := Dummy + 24 * 60;
     end;
     If (LoByte AND $20 = $20) then
		     Dummy := Dummy - OffSet * 30
		     else
		     Dummy := Dummy + Offset * 30;
     RDS_H := Dummy DIV 60;
     RDS_M := Dummy MOD 60;
     end;
     Str(RDS_H:2,Tempo);
     If (Tempo = '24') then
		     Tempo := ' 0';
     RDStime := Tempo + ':';
     Str(RDS_M:2,Tempo);
     If (Tempo[1] = ' ') then
		     Tempo[1] := '0';
     RDStime := RDStime + Tempo;
     SetLength(RDStime, 5);
     end;
     End;  {maketime}
     */
     
     
/*************************** AF Handling *****************************/
static int InvalidAFdata(int HiByte, int LoByte) {
	static int HiByte_prev = -1;
	static int LoByte_prev = -1;
	static int _InvalidAFdata = 1;
	
	if ( (HiByte_prev != HiByte) || (LoByte_prev != LoByte) ) {
		_InvalidAFdata =
			(HiByte == 0x00) | (LoByte == 0x00) |
			((HiByte >= 0xCE) && (HiByte<= 0xDF)) | ((LoByte >= 0xCE) && (LoByte<= 0xDF)) |
			((HiByte >= 0xFB) && (HiByte<= 0xFF)) | ((LoByte >= 0xFB) && (LoByte<= 0xFFF)) |
			((HiByte == 0xE0) && (LoByte == 0xCD)) |
			((LoByte == 0xE0) && (HiByte == 0xCD)) |
			(HiByte == LoByte);
	} 

	return _InvalidAFdata;
}

static void CheckMethod(void) {

}

static void PutFrequencyList(void) {

}

static void PutChkList(void) {

}


#endif
static int rdsBlockId (_status_field *pt) {

	int ret;
	switch( pt->field.bidn ) {
//  Case ((Status AND $1C) SHR 2) of
		case 0 : ret = eAType; break;
		case 1 : ret = eBType; break;
		case 2 : ret = eCType; break;
		case 3 : ret = eDType; break;
		case 4 : ret = eC_Type; break;
		case 5 : ret = eEType; break;
		case 6 : ret = eE_Type; break;
		case 7 : ret = eNone; break;
		default: ret = eNone; break;
	}
	
	return ret;

}

static int rdsError (_status_field *pt) {
	return pt->field.ebn;
}

static int rdsIsInSync (_status_field *pt) {
	return (pt->field.sync == 1);
}

static int rdsIsInReset (_status_field *pt) {
	return (pt->field.rstd == 1);
}

static int rdsHasOverflow (_status_field *pt) {
	return (pt->field.dovf == 1);
}

/************************ Block A processing **********************************/
static void ProcessBlockA(int HiByte, int LoByte) {
// PI code

	rdsbasic.pi = (HiByte << 8) + LoByte;
//	if (B_Flag) {
//		EONdata  = rdsbasic.pi;
//		EONarmed = TRUE;
//	}
	return;
}

/************************ Block B processing **********************************/
static void B0_block(int HiByte, int LoByte) {
//{ alternative frequencies and program service name }
//{ A0 block }
//	rdsbasic.tp  = ((HiByte & 0x04) == 0x04);
//	rdsbasic.pty = ((HiByte & 0x03) << 0x03) +
//                 ((LoByte & 0xE0) >> 0x05);
	DataAddress = LoByte & 0x1F;
	
//	printk( KERN_INFO "B0_block, DataAddress=%d - LoByte=%d\n",DataAddress, LoByte);
//	rdsbasic.ta  = ((LoByte & 0x10) == 0x10);
//	rdsbasic.ms  = ((LoByte & 0x08) == 0x08);
//	UpdateDecoderInfo();
//{B0_block}
}

#if 0
static void B2_block(void) {
//{ radio text }
	TextFlag = ((DataAddress & 0x10) == 0x10);
	if (TextFlag != OlTextFlag) {
		ClearRadioText();
		OlTextFlag = TextFlag;
	}
// {B2_block}

}

static void B10_block(void) {
//{ pty name }
	PTYNflag = ((DataAddress & 0x10) == 0x10);
	if (PTYNflag != OlPTYNflag) {
		ClearPTYname();
		OlPTYNflag = PTYNflag;
	}
//End;  {B10_block}
}

static void B14_block(int LoByte) 
{
//{ enhanced other networks }
	rdsbasic.eon = TRUE;
	TP_ON        = ((LoByte & 0x10) == 0x10);
	TA_ON        = FALSE;

	if (B_Flag) {
		TA_ON = ((LoByte & 0x08) == 0x08);
		EONselector = 0xFF;
	} else
		EONselector  =  LoByte & 0x0F;
//End;  {B14_block}
}
#endif

static void ProcessBlockB(int HiByte, int LoByte) {

//  Group        :=  (HiByte AND $F0) SHR 4;
	Group = (HiByte & 0xF0) >> 4;
//  B_Flag       :=  (HiByte AND $08) = $08;
	B_Flag =  ((HiByte & 0x08) == 0x08);
//   rdsbasic.TP  := ((HiByte AND $04) = $04);
//	rdsbasic.tp = ((HiByte & 0x04) == 0x04);
//   rdsbasic.PTY := ((HiByte AND $03) SHL $03) + ((LoByte AND $E0) SHR $05);
//	rdsbasic.pty = ((HiByte & 0x03) << 0x03) + ((LoByte & 0xE0) >> 0x05);

//   DataAddress  :=   LoByte AND $1F;
	DataAddress = LoByte & 0x1F;
//	printk( KERN_INFO "ProcessBlockB ,DataAddress=%d,LoByte=%d,B_Flag=%d\n",DataAddress,LoByte,B_Flag);
//   TA_ON        := FALSE;
//	TA_ON = FALSE;

	if (B_Flag) {
		switch (Group) {
		case 0 :
			rds_group_start_psn_timer();
			B0_block(HiByte,LoByte); 
			break;
//		case 2 :  B2_block(); break;
//		case 14:  B14_block(LoByte); break;
		default: break;
		}
	} else {
		switch (Group) {
		case 0 :
			A0Groups++;
			rds_group_start_psn_timer();
			B0_block(HiByte,LoByte);
			break;
//		case 2 :  B2_block(); break;
//		case 10 :  B10_block(); break;
//		case 14 :  B14_block(LoByte); break;
		default: break;
		}

	}

//	EONarmed = FALSE;

}

#if 0
/************************ Block C processing **********************************/
static void C2_block(int HiByte, int LoByte)
{ //radio text }
	FillRadioText(1,HiByte,LoByte);
}

static void C0_block(int HiByte, int LoByte) {
//{ alternative frequencies }
	if (InvalidAFdata(HiByte,LoByte))
		return;

	if (!B_Method)
		CheckMethod();

	if (B_Method)
		PutFrequencyList(AFlist);
	else
	{
		PutFrequency(AFlist[0]);
		PutChkList(ChkList);
	}
//End;  {C0_block}
}
Procedure C1_Block;
{ slow labelling codes}
Var Variant : Integer;
Begin
  Variant := (HiByte AND $70) SHR 4;
  With SLC do
    begin
      LA := ((HiByte AND $80) = $80);
      Case Variant of
        0 : begin OPC :=  HiByte AND $0F; ECC := LoByte; end;
        1 : TMCid    := ((HiByte AND $0F) SHL 8) + LoByte;
        2 : PagingId := ((HiByte AND $0F) SHL 8) + LoByte;
        3 : LC       := ((HiByte AND $0F) SHL 8) + LoByte;
        7 : EWSid    := ((HiByte AND $0F) SHL 8) + LoByte;
      end;
    end;
End;  {c1_block}



Procedure C4_block;
{ date }
Begin                    { AC4 Group }
  RDStime_Flag := (LoByte AND $01) = $01;
  MJD          := (DataAddress AND $03) SHL 15 +
                   HiByte SHL 7 +
                  (LoByte AND $FE) SHR 1;
  MakeDate;
End;  {C4_block}

Procedure C10_Block;
{program type name}
Begin
  ProgramTypeName(0);
End; {c10_block}

Procedure C14_block;
{ eon }
Begin                    { C14 block }
  EONdata  := HiByte SHL 8 + LoByte;
  EONarmed := TRUE;
End;  {C14_block}


static void ProcessBlockC(int HiByte, int LoByte) {

	if (B_Flag)  // second pi transmission in third block
		ProcessBlockA(HiByte, LoByte);
	else 
	{
		switch (Group) {
// 			case 0 :  C0_block(); break;	// alternative frequencies
// 			case 1 :  C1_Block(); break;	// slow labelling codes
			case 2 :  C2_block(HiByte, LoByte); break;	// radio text
// 			case 4 :  C4_block(); break;	// clock time and date
// 			case 10 :  C10_block(); break;	// program type name
// 			case 14 :  C14_block(); break;	// eon data packet
		}
	}
//End;  {processblockc}

}
#endif
#if 1
/*(************************ Block D processing **********************************)*/

static void D0_block(int HiByte, int LoByte)
{
//program service name 
//	printk( KERN_INFO "rdsDecodeBlock ,BlockType=%d \n",BlockType);
	ProgramServiceName(HiByte,LoByte);
}

// Procedure D1_block;
// { program item number}
// Begin
//   ProgramItemNumber(SLC.PINcode);
// End; { d1_block}
#if 0
static void D2_block(int HiByte, int LoByte)
{
	 //radio text
	FillRadioText(2,HiByte,LoByte);
}
// static void D4_block (void)
// { 
// 	//clock time 
//   if(!B_Flag)
//     MakeTime();
// }

static void D10_block(int HiByte, int LoByte)
{
	//program type name
	ProgramTypeName(1,HiByte,LoByte);
}

// Procedure D14_block;
// { eon }
// Begin                    { D14 block }
//   PI_ON := (HiByte SHL 8) + LoByte;
//   If B_Flag then
//     UpdateEONdata
//   else
//     begin
//       If (PI_ON = rdsbasic.PI) then
//         If (EONselector = $0C) then     // linkage information current frequency
//           rdsbasic.OwnLink := EONdata
//         else
//           UpdateEONdata
//       else
//         UpdateEONdata;
//     end
// End;  {D4_block}
#endif 

static void ProcessBlockD( int HiByte, int LoByte )
{
	if ( B_Flag ) {
		switch ( Group )
		{
			case 0:
				D0_block(HiByte,LoByte);
				break;
			case 2:
//				D2_block(HiByte,LoByte);
				break;
				// 		  case 14 :
				// 			  D14_block();
				// 			  break;
			case 15:
				B0_block(HiByte,LoByte);
				break;
			default:
				break;
		}
	} else {
		switch ( Group )
		{
			case 0:
				D0_block(HiByte,LoByte);
				break;
				// 		  case 1:
				// 			  D1_Block();
				// 			  break;
			case 2:
//				D2_block(HiByte,LoByte);
				break;
// 			case 4:
// 				D4_block();
// 				break;
			case 10:
//				D10_block(HiByte,LoByte);
				break;
				// 		  case 14 :
				// 			  D14_block();
				// 			  break;
			default:
				break;
		}
	}
}
#endif

static void rdsDecodeBlock(int BlockType, int HiByte, int LoByte) 
{
//	printk( KERN_INFO"rdsDecodeBlock ,BlockType=%d \n",BlockType);
	switch (BlockType) {
		case eAType	: ProcessBlockA(HiByte,LoByte); break;
		case eBType	: ProcessBlockB(HiByte,LoByte); break;
//		case eCType	: ProcessBlockC(HiByte,LoByte); break;
		case eC_Type	: ProcessBlockA(HiByte,LoByte); break;
		case eDType	: ProcessBlockD(HiByte,LoByte); break;
		case eEType	:
		case eE_Type	:
		case eNone	:
		break;
	}

	return;
}

static int rdsProcess(_status_field *pt_rds_data, int HiByte, int LoByte) {
	
	static _status_field pstatus;
	static int pdata1 = -1;
	static int pdata2 = -1;

	if ( (pstatus.reg == pt_rds_data->reg) && (pdata1 == HiByte) && (pdata2 == LoByte) ){
// 		printk( KERN_INFO "rdsProcess ,same values -> return eNone \n");
		return eNone;
	}
	
	pstatus.reg = pt_rds_data->reg;
	pdata1 = HiByte;
	pdata2 = LoByte;

//   // convert status byte to separate values
	BlockType  = rdsBlockId(&pstatus);
	Error      = rdsError(&pstatus);
	InSync     = rdsIsInSync(&pstatus);
	InReset    = rdsIsInReset(&pstatus);
	InOverflow = rdsHasOverflow(&pstatus); //(doStatus AND $40) = $40;

	if (!InSync || InReset){
// 		 printk( KERN_INFO "rdsProcess ,InSync=%d InReset=%d -> return eNone \n",InSync,InReset);
		return eNone;
	}
	
	if (!Error){
 		// printk(KERN_INFO "rdsProcess -> rdsDecodeBlock type=%d \n",BlockType);
		rdsDecodeBlock(BlockType,HiByte,LoByte);
 		// printk(KERN_INFO "RDS Group = %d\n",Group);
 		// printk(KERN_INFO "RDS DataAddress = %d\n",DataAddress);
 		// printk(KERN_INFO "== Prg service name : %s\n",rdsbasic.psn);
 		// printk(KERN_INFO "== RADIO TEXT = %s\n",rdsbasic.rt);
 		// printk(KERN_INFO "== DATE : %s\n",rdsbasic.rdsdate);	// rds date
 		// printk(KERN_INFO "== TIME : %s\n",rdsbasic.rdstime);	// rds time
 		// printk(KERN_INFO "== PI : %d\n",rdsbasic.pi);		// pi code
 		// printk(KERN_INFO "== PTY : %d\n",rdsbasic.pty);		// program type code
 		// printk(KERN_INFO "== PTY NAME : %s\n",rdsbasic.ptyn);
 		// printk(KERN_INFO "== DI : %d\n",rdsbasic.di);		// decoder information
 		// printk(KERN_INFO "== MS : %d\n",rdsbasic.ms);		// music/speech flag
 		// printk(KERN_INFO "== TP : %d\n",rdsbasic.tp);		// traffic program
 		// printk(KERN_INFO "== TA : %d\n",rdsbasic.ta);		// trafic announcement
 		// printk(KERN_INFO "== EON : %d\n",rdsbasic.eon);		// enhanced other networks
 		// printk(KERN_INFO "\n");
	}

	return BlockType;
}

void Process_RDSdata(radio_tea5766_rds_regs_t *pt_rds_data) 
{
// prepare variables for rds processing
	static int _now_data = -1;
	static int _prev_data;
	static int HiByte,LoByte;
	_status_field status;
	_rdsw1_field _rdsw1;
	_rdsr1_field _rdsr1;


//	NowData := BLn SHL 16 + BPn;                         // store current data
	_now_data = ((pt_rds_data->rdsr2)<<16) + (pt_rds_data->rdsr3);

//	If (NowData = PrevData) then                          // skip same reading
//        Exit;
	if ( _now_data == _prev_data)
		return;
	
	_prev_data = _now_data;

	_rdsw1.reg = pt_rds_data->rdsw1;
	_rdsr1.reg = pt_rds_data->rdsr1;

	if ( _rdsw1.field.dac == DAVC ) {
// 		 printk( KERN_INFO "Process_RDSdata process 2 blocs\n");
          // Previous block
/*
          Status := EPBn +                                              // Error
                    BPIDn SHL 2 +                                    // Block id
                    INTEGER(RSTD)  SHL 5 +                              // Reset
                    INTEGER(DOVF)  SHL 6 +                           // Overflow
                    INTEGER(SYNC)  SHL 7;                             // In Sync
          HiByte := BPn AND $FF00 SHR 8;                            // High byte
          LoByte := BPn AND $00FF;                                   // Low byte
*/
		status.field.ebn	= _rdsr1.field.epb;
		status.field.bidn	= _rdsr1.field.bpid;
		status.field.rstd  	= _rdsr1.field.rstd;
		status.field.dovf	= _rdsr1.field.dovf;
		status.field.sync	= _rdsr1.field.sync;
		HiByte = ((pt_rds_data->rdsr3) & 0xFF00)>>8;
		LoByte = ((pt_rds_data->rdsr3) & 0x00FF);

		rdsProcess(&status, HiByte, LoByte);           // send to rds processor
	} else {
// 		 printk( KERN_INFO "Process_RDSdata process 1 bloc\n");
	}
      // Last block
/*
      Status := ELBn +                                                  // Error
                BLIDn SHL 2 +                                        // Block id
                INTEGER(RSTD) SHL 5 +                                   // Reset
                INTEGER(DOVF) SHL 6 +                                // Overflow
                INTEGER(SYNC) SHL 7;                                  // In Sync
      HiByte := BLn AND $FF00 SHR 8;                                // High byte
      LoByte := BLn AND $00FF;                                       // Low byte
*/
	status.field.ebn	= _rdsr1.field.elb;
	status.field.bidn	= _rdsr1.field.blid;
	status.field.rstd  	= _rdsr1.field.rstd;
	status.field.dovf	= _rdsr1.field.dovf;
	status.field.sync	= _rdsr1.field.sync;
	HiByte = ((pt_rds_data->rdsr2) & 0xFF00)>>8;
	LoByte = ((pt_rds_data->rdsr2) & 0x00FF);

	rdsProcess(&status, HiByte, LoByte);               // send to rds processor
}


int RDS_get_basic_data(_trdsbasic_t * pdata)
{
	memcpy(pdata,&rdsbasic,sizeof(_trdsbasic_t));
	return(0);
}


