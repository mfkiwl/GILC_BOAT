#ifndef __MEM_H
#define __MEM_H

#ifdef STM32
#define GILC_MEM_USE
#endif

#ifdef __cplusplus
extern "C" {
#endif				/*__cplusplus*/

/*mem 信息结构体*/
typedef struct
{
  unsigned char used_pct;
  unsigned char frag_pct;
  unsigned int total_size;
  unsigned int free_cnt;
  unsigned int free_size;
  unsigned int free_biggest_size;
  unsigned int used_cnt;
}M_MEM_INFO;


unsigned int m_getMemSize(const void * data);
void *m_malloc(unsigned int size);
void m_free(const void * data);
void m_memPoolInit(void *buf,unsigned int len);
void m_getMemInfo(M_MEM_INFO * mon_p);

#ifdef GILC_MEM_USE
#define gilc_malloc(size)          m_malloc(size)
#define gilc_free(addr)            m_free(addr)
#define gilc_getMemSize(p)         m_getMemSize(p)
#define gilc_MemPoolInit(pBuf,len) m_memPoolInit(pBuf,len)
#define gilc_getMemInfo(mon_p)     m_getMemInfo(mon_p)
#else
#define gilc_malloc(size)  malloc(size)
#define gilc_free(addr)    free(addr)
#endif

#ifdef __cplusplus
}
#endif			/*__cplusplus*/

#endif
