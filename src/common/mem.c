#include <string.h>
#include "mem.h"
#include "log.h"
#include "hc_type.h"

#define MEM_UNIT unsigned int /*32位，有的64位的需要改成u8*/

#define MATH_MAX(a,b)      (a > b ? a : b)
#define MATH_MIN(a,b)      (a < b ? a : b)
#define MATH_DIV(a,b)      (b ? (a / b) : 0)
#define MATH_MOD(a,b)      (b ? (a % b) : 0)
#define MATH_ABS(x)        ((x) > 0 ? (x) : (-(x)))

typedef enum
{
	MEM_START = 0,
	MEM_GILC = 0,
	MEM_END
}MEM_POOL_NAME;

/* MEM头结构体 */
typedef union {
  struct {
      MEM_UNIT used: 1;       /*1: if the entry is used*/
      MEM_UNIT d_size: 31;    /*Size off the data (1 means 4 bytes) */
  };
  MEM_UNIT header;            /*The header (used + d_size)*/
} MEM_HEAD_STRUCT;

/* MEM动态分配结构体 */
typedef struct {
	MEM_HEAD_STRUCT header;
	unsigned char first_data;        /*First data byte in the allocated data (Just for easily create a pointer)*/
} MEM_ALLOC_STRUCT;

/*mem 配置参数结构体*/
typedef struct
{
	MEM_POOL_NAME memname;
	unsigned char *work_mem;    /*Work memory for allocation*/
	unsigned int totalsize;
	unsigned char mutex_name;
}MEM_CFG;

/*mem 信息结构体*/
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
}MEM_INFO;

static unsigned int get_mem_size(const void * data);
static void *mem_alloc(MEM_POOL_NAME mem_name,unsigned int size);
static void mem_free(MEM_POOL_NAME mem_name,const void * data);
static void create_mem_pool(MEM_POOL_NAME mem_name,void *buf,unsigned int len);
static void get_mem_info(MEM_POOL_NAME mem_name,MEM_INFO * mon_p);

static u4 zero_mem;       /*Give the address of this variable if 0 byte should be allocated*/
static void ent_trunc(MEM_ALLOC_STRUCT * e, u4 size);
static void *ent_alloc(MEM_ALLOC_STRUCT * e, u4 size);
static MEM_ALLOC_STRUCT * ent_get_next(MEM_POOL_NAME mem_name,MEM_ALLOC_STRUCT * act_e);

static MEM_CFG mem_cfg[MEM_END] = 
{
	{
		.memname = MEM_GILC,
		.work_mem = (void *)0,
		.totalsize = 0,
	}
};

/**
 * Give the next entry after 'act_e'
 * @param act_e pointer to an entry
 * @return pointer to an entry after 'act_e'
 */
static MEM_ALLOC_STRUCT * ent_get_next(MEM_POOL_NAME mem_name,MEM_ALLOC_STRUCT * act_e)
{	
	MEM_ALLOC_STRUCT * next_e = NULL;
	if(act_e == NULL) 
	{ /*NULL means: get the first entry*/
		next_e = (MEM_ALLOC_STRUCT *) mem_cfg[mem_name].work_mem;
	}
	else
	{ /*Get the next entry */
		u1 * data = &act_e->first_data;
		next_e = (MEM_ALLOC_STRUCT *)&data[act_e->header.d_size];
		if(&next_e->first_data >= &mem_cfg[mem_name].work_mem[mem_cfg[mem_name].totalsize]) 
			next_e = NULL;
	}
	return next_e;
}
/**
 * Try to do the real allocation with a given size
 * @param e try to allocate to this entry
 * @param size size of the new memory in bytes
 * @return pointer to the allocated memory or NULL if not enough memory in the entry
 */
static void *ent_alloc(MEM_ALLOC_STRUCT * e, u4 size)
{
	void * alloc = NULL;
	/*If the memory is free and big enough then use it */
	if(e->header.used == 0 && e->header.d_size >= size) {
		/*Truncate the entry to the desired size */
		ent_trunc(e, size),
		e->header.used = 1;
		/*Save the allocated data*/
		alloc = &e->first_data;
	}
	return alloc;
}

/**
 * Truncate the data of entry to the given size
 * @param e Pointer to an entry
 * @param size new size in bytes
 */
static void ent_trunc(MEM_ALLOC_STRUCT * e, u4 size)
{
	/*Round the size up to 4*/
	if(size & 0x3)
	{
		size = size & (~0x3);
		size += 4;
	}
	/*Don't let empty space only for a header without data*/
	if(e->header.d_size == size + sizeof(MEM_HEAD_STRUCT)) {
		size = e->header.d_size;
	}
	/* Create the new entry after the current if there is space for it */
	if(e->header.d_size != size) {
		u1 * e_data = &e->first_data;
		MEM_ALLOC_STRUCT * after_new_e = (MEM_ALLOC_STRUCT *)&e_data[size];
		after_new_e->header.used = 0;
		after_new_e->header.d_size = e->header.d_size - size - sizeof(MEM_HEAD_STRUCT);
	}
	/* Set the new size for the original entry */
	e->header.d_size = size;
}

/**
 * get the rest size of memroy 
 * @param e Pointer to an entry
 * @param size in bytes
 */
static u4 get_mem_size(const void * data)
{
	if(data == NULL) 
		return 0;
	if(data == &zero_mem) 
		return 0;
	MEM_ALLOC_STRUCT * e = (MEM_ALLOC_STRUCT *)((u1 *) data - sizeof(MEM_HEAD_STRUCT));
	return e->header.d_size;
}

/**
 * Allocate a memory dynamically
 * @param size size of the memory to allocate in bytes
 * @return pointer to the allocated memory
 */
static void *mem_alloc(MEM_POOL_NAME mem_name,u4 size)
{
	if(size == 0) {
		return &zero_mem;
	}
	/*Round the size up to 4*/
	if(size & 0x3) 
	{
		size = size & (~0x3);
		size += 4;
	}

  void * alloc = NULL;
	MEM_ALLOC_STRUCT * e = NULL;
	//Search for a appropriate entry
	do {
		//Get the next entry
		e = ent_get_next(mem_name,e);
		/*If there is next entry then try to allocate there*/
		if(e != NULL) {
			alloc = ent_alloc(e, size);
		}
		//End if there is not next entry OR the alloc. is successful
	} while(e != NULL && alloc == NULL);
	if(alloc == NULL)
	{		
		gilc_log("alloc fail\r\n");
	}
	return alloc;
}

/**
 * Free an allocated data
 * @param data pointer to an allocated memory
 */
static void mem_free(MEM_POOL_NAME mem_name,const void * data)
{
    if(data == &zero_mem) return;
    if(data == NULL) return;

	  MEM_ALLOC_STRUCT * e = (MEM_ALLOC_STRUCT *)((u1 *) data - sizeof(MEM_HEAD_STRUCT));
    e->header.used = 0;
    /* Make a simple defrag.
     * Join the following free entries after this*/
    MEM_ALLOC_STRUCT * e_next;
    e_next = ent_get_next(mem_name,e);
    while(e_next != NULL) 
		{
        if(e_next->header.used == 0) 
				{
            e->header.d_size += e_next->header.d_size + sizeof(e->header);
        } 
				else
				{
            break;
        }
        e_next = ent_get_next(mem_name,e_next);
    }
}
 
/**
 * Give information about the work memory of dynamic allocation
 * @param mon_p pointer to a dm_mon_p variable,
 *              the result of the analysis will be stored here
 */
static void get_mem_info(MEM_POOL_NAME mem_name,MEM_INFO * mon_p)
{
	/*Init the data*/
	memset(mon_p, 0, sizeof(MEM_INFO));
	MEM_ALLOC_STRUCT * e;
	e = NULL;
	e = ent_get_next(mem_name,e);
	while(e != NULL)  {
		if(e->header.used == 0) {
			mon_p->free_cnt++;
			mon_p->free_size += e->header.d_size;
			if(e->header.d_size > mon_p->free_biggest_size) {
				mon_p->free_biggest_size = e->header.d_size;
			}
		} else {
			mon_p->used_cnt++;
		}
		e = ent_get_next(mem_name,e);
	}
	mon_p->total_size = mem_cfg[mem_name].totalsize;;
	mon_p->used_pct = 100 - (100U * mon_p->free_size) / mon_p->total_size;
	mon_p->frag_pct = (u4)mon_p->free_biggest_size * 100U / mon_p->free_size;
	mon_p->frag_pct = 100 - mon_p->frag_pct;
}
/*mem_name = MEM_GILC，buf就是我传给你首地址，len就是我传给你的长度*/
static void create_mem_pool(MEM_POOL_NAME mem_name,void *buf,u4 len)
{
	mem_cfg[mem_name].work_mem = (u1 *) buf;
	mem_cfg[mem_name].totalsize = len;
	MEM_ALLOC_STRUCT * full = (MEM_ALLOC_STRUCT *)mem_cfg[mem_name].work_mem;
	full->header.used = 0;
	/*The total mem size id reduced by the first header and the close patterns */
	full->header.d_size = len - sizeof(MEM_HEAD_STRUCT);
}


/**
 * get the rest size of memroy 
 * @param e Pointer to an entry
 * @param size in bytes
 */
unsigned int m_getMemSize(const void * data)
{
	return get_mem_size(data);
}

/**
 * Allocate a memory dynamically
 * @param size size of the memory to allocate in bytes
 * @return pointer to the allocated memory
 */
void *m_malloc(unsigned int size)
{
	return mem_alloc(MEM_GILC,size);
}

/**
 * Free an allocated data
 * @param data pointer to an allocated memory
 */
void m_free(const void * data)
{
	mem_free(MEM_GILC,data);
}
 
/**
 * Give information about the work memory of dynamic allocation
 * @param mon_p pointer to a dm_mon_p variable,
 *              the result of the analysis will be stored here
 */
void m_getMemInfo(M_MEM_INFO * mon_p)
{
	get_mem_info(MEM_GILC,(MEM_INFO *)mon_p);
}

/*mem_name = MEM_GILC，buf就是我传给你首地址，len就是我传给你的长度*/
void m_memPoolInit(void *buf,unsigned int len)
{
	create_mem_pool(MEM_GILC,buf,len);
}

