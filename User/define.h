/**
  ******************************************************************************
  * @file        define.h
  * @author      Johnny
  * @version     v0.0
  * @date        2021/9/8
  * @brief       
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2021 Johnny</center></h2>
  *
  *
  ******************************************************************************
  */

#ifndef _DEFINE_H_
#define _DEFINE_H_

#define	DEBUG_MODE				//don't check HW Protect 

//-------------------------------------------------
typedef unsigned char			BYTE;
typedef	char					CHAR;
typedef	unsigned short int		WORD;
typedef short int				INT;
typedef unsigned long int		DWORD;
typedef long int				LONG;
typedef unsigned char			BOOL;
typedef unsigned long long int 	DDWORD;
typedef long long int			DLONG;

typedef union{
	BYTE	b[2];
	WORD	i;
	INT		si;
}tIbyte;
	
typedef union{
	BYTE	b[4];
	WORD	i[2];
	DWORD	l;
	LONG	sl;
}tLbyte;

typedef union{
	BYTE	b[8];
	WORD	i[4];
	DWORD	l[2];
	DDWORD  ll;
	DLONG	sll;
}tLLbyte;

#define	GET_BYTE(addr)		((BYTE)(*(BYTE *)addr))	
//#define	GET_WORD(addr)		((WORD)(*(WORD *)addr))	
//#define	GET_DWORD(addr)		((DWORD)(*(DWORD *)addr))

#define	GET_WORD(addr)		((((WORD)(*(BYTE *)(addr+1)))<<8) + (((WORD)(*(BYTE *)(addr+0)))))
#define	GET_DWORD(addr)		((((DWORD)(*(BYTE *)(addr+3)))<<24) + (((DWORD)(*(BYTE *)(addr+2)))<<16) + (((DWORD)(*(BYTE *)(addr+1)))<<8) + (((DWORD)(*(BYTE *)(addr+0)))))


#endif

