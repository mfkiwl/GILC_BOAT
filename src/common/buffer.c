#include <stdlib.h>
#include "buffer.h"

/********************************************************************************
*Function Name: create_buffer
*Description:  ����һ��buffer�ռ�,��ʼ����Ϊ0
*Input Params: @iSize - buffer��С
*Out Params:   none
*Return Value: ���ظ�buffer�Ĺ���ṹ��
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
BUFFER_STRUCT *create_buffer( HC_INT32 iSize )
{
	//BUFFER_STRUCT *pstBuf = ( BUFFER_STRUCT * ) calloc( 1, sizeof( BUFFER_STRUCT ) );
	//pstBuf->m_pucBuf = ( HC_UINT8 *) calloc( iSize, sizeof(HC_UINT8) );
	BUFFER_STRUCT *pstBuf = ( BUFFER_STRUCT * ) buf_malloc( sizeof( BUFFER_STRUCT ) );
	if(!pstBuf) 
	{
		loge("malloc err\r\n");
	}
	pstBuf->m_pucBuf = ( HC_UINT8 *) buf_malloc( iSize );
	if(!pstBuf->m_pucBuf) 
	{
		loge("malloc err\r\n");
	}	
	pstBuf->m_iMaxSize = iSize;
	pstBuf->m_iEnd = pstBuf->m_iStart =  pstBuf->m_iSize =0;
	return pstBuf;
}
/********************************************************************************
*Function Name: release_buffer
*Description:  �ͷ���create_buffer�����Ŀռ�
*Input Params: @pstBuf - ��Ӧ��buffer����ṹ��
*Out Params:   none
*Return Value: none
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/

HC_VOID release_buffer( BUFFER_STRUCT* pstBuf )
{
	//free( pstBuf->m_pucBuf );
	//free( pstBuf );
	buf_free( pstBuf->m_pucBuf );
	buf_free( pstBuf );
}

/********************************************************************************
*Function Name: add_to_buffer
*Description:  ��buffer�м���һ�γ��ȵ�����
*Input Params: @pstBuf - ��Ӧ��buffer����ṹ��
			   @pucData - Ҫ�������ݵ���ʼ��ַ
			   @iLen - Ҫ�������ݵĳ���
*Out Params:   none
*Return Value: 0 - success, -1 -  fail
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
HC_INT32 add_to_buffer( BUFFER_STRUCT* pstBuf, HC_UINT8 * pucData, HC_INT32 iLen )
{
	HC_INT32 i = 0;

	if (iLen + pstBuf->m_iSize > pstBuf->m_iMaxSize)
	{
		return HC_ERR;
	}
	
	for (i = 0; i < iLen; ++i)
	{
		if ( ( pstBuf->m_iStart + 1 ) % ( pstBuf->m_iMaxSize ) == pstBuf->m_iEnd )
		{
			return HC_ERR;
		}
		pstBuf->m_pucBuf[pstBuf->m_iStart] = pucData[i];
		//printf("%c",pstBuf->m_pucBuf[pstBuf->m_iStart]);
		pstBuf->m_iStart = (pstBuf->m_iStart + 1) % (pstBuf->m_iMaxSize);
		(pstBuf->m_iSize)++;
	}
	//printf("buf size: %d\r\n", pstBuf->m_iSize);
	return HC_OK;
}

/********************************************************************************
*Function Name: remove_from_buffer
*Description:  ��buffer��ɾ��һ�γ��ȵ�����
*Input Params: @pstBuf - ��Ӧ��buffer����ṹ��
			   @iLen - Ҫ�������ݵĳ���
*Out Params:   none
*Return Value: 0 - success, -1 -  fail
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
HC_INT32 remove_from_buffer(BUFFER_STRUCT* pstBuf, HC_INT32 iLen)
{
	if (iLen > pstBuf->m_iSize)
	{
		iLen = pstBuf->m_iSize;
	}
	pstBuf->m_iEnd = (pstBuf->m_iEnd + iLen) % (pstBuf->m_iMaxSize);
	pstBuf->m_iSize -= iLen;
	return HC_OK;
}

/********************************************************************************
*Function Name: get_from_buffer
*Description:  ��buffer��ȡ��һ�γ��ȵ�����
*Input Params: @pstBuf - ��Ӧ��buffer����ṹ��
			   @iLen - Ҫ�������ݵĳ���
*Out Params:   @pucData - ���ȡ��������
*Return Value: ���ض�ȡ�����ݳ���
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
HC_INT32 get_from_buffer(BUFFER_STRUCT* pstBuf, HC_UINT8 *pucData, HC_INT32 iLen)
{
	HC_INT32 iCnt = 0;
	if (iLen > pstBuf->m_iSize)
	{
		iLen = pstBuf->m_iSize;
	}
	while (iCnt < iLen)
	{
		pucData[iCnt] = pstBuf->m_pucBuf[(pstBuf->m_iEnd + iCnt) % (pstBuf->m_iMaxSize)];
		iCnt++;
	}
	return iLen;
}

/********************************************************************************
*Function Name: get_from_buffer_withremove
*Description:  ��buffer��ȡ��һ�γ��ȵ�����,ͬʱɾ��ԭ����
*Input Params: @pstBuf - ��Ӧ��buffer����ṹ��
			   @iLen - Ҫ�������ݵĳ���
*Out Params:   @pucData - ���ȡ��������
*Return Value: 0 - success, -1 -  fail
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
HC_INT32 get_from_buffer_withremove(BUFFER_STRUCT* pstBuf, HC_UINT8 *pucData, HC_INT32 iLen)
{
	HC_INT32 iCnt = 0;
	if ( iLen > pstBuf->m_iSize )
		iLen = pstBuf->m_iSize;
	while ( iCnt < iLen )
	{
		pucData[iCnt] = pstBuf->m_pucBuf[(pstBuf->m_iEnd + iCnt) % (pstBuf->m_iMaxSize)];
		iCnt++;
	}
	remove_from_buffer(pstBuf, iLen);
	return iLen;
}

/********************************************************************************
*Function Name: clean_buffer
*Description:  ���buffer
*Input Params: @pstBuf - ��Ӧ��buffer����ṹ��
*Out Params:   @pstBuf - ��Ӧ��buffer����ṹ��
*Return Value: 0 - success, -1 -  fail
*********************************************************************************
*Modified time        Version      Modified by       Modified Content
*  2016/9/28            V1.0          lcy                create    
*********************************************************************************/
HC_VOID clean_buffer(BUFFER_STRUCT *pstBuf)
{
	pstBuf->m_iEnd = pstBuf->m_iStart = pstBuf->m_iSize = 0;
}

