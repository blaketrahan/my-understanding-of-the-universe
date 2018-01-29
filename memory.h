#ifndef _GAME_MEMORY_H_
#define _GAME_MEMORY_H_

#include <stdlib.h>     /* malloc, free, rand */ 
#include <cstring>      /* memset */ 
#include <stdint.h>

#define domestic static
#define global_variable static
#define local_persist static

#define Kilobytes(Value) ((Value) * 1024)
#define Megabytes(Value) (Kilobytes(Value)*1024)
#define Gigabytes(Value) (Megabytes(Value)*1024)

typedef int8_t  s1;   // 8 bits, 1 bytes
typedef int16_t s2;  // 16 bits, 2 bytes
typedef int32_t s4;
typedef int64_t s8;  // 64 bits, 8 bytes

typedef int32_t b4;   // 32 bits, 8 bytes

typedef uint8_t  u1;
typedef uint16_t u2;
typedef uint32_t u4;
typedef uint64_t u8;

typedef float  f4;
typedef double f8;

// -- OLD TYPEDEFS --
typedef int8_t  int8;   // 8 bits, 1 bytes
typedef int16_t int16;  // 16 bits, 2 bytes
typedef int32_t int32;  // 32 bits, 4 bytes
typedef int64_t int64;  // 64 bits, 8 bytes
typedef int32 bool32;   // 32 bits, 8 bytes
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64; 
typedef float  d32;
typedef double d64;

#define MEM_DEBUG

struct GameMemory
{
    b4  isInitialized;
    uint64_t  PermanentStorageSize;
    void* PermanentStorage;
    uint64_t  current; 
    
    uint64_t  TransientStorageSize;
    void* TransientStorage;
    uint64_t  transient_current;  
};

global_variable GameMemory memory;

// Empty transient memory AND zero out storage
inline void empty_transient(GameMemory &memory)
{
    memset(memory.TransientStorage,0,memory.TransientStorageSize);
    memory.transient_current = 0;
} 

// Empty transient memory without zeroing storage
inline void empty_transient_soft(GameMemory &memory)
{ 
    memory.transient_current = 0;
} 

// Allocate permanent memory
void* alloc(GameMemory &memory, uint64_t n)
{
    memory.current += n;
    // cast to unsigned byte so i can increment it by single bytes
    return ( ((u1*)memory.PermanentStorage) + memory.current - n);
} 

// Allocate transient memory
void* alloc_transient(GameMemory &memory, std::size_t n)
{
    memory.transient_current += n;
    // std::cout << memory.transient_current << std::endl;
    return ( ((u1*)memory.TransientStorage) + memory.transient_current - n);
}  

// Print out structure
void check_storage(GameMemory &memory)
{
    const s4 MEGABYTES_SIZE = 8 * 1024 * 1024;
    std::cout << std::endl;
    std::cout << "/------------ Game Memory -----------------------" << std::endl;  
    std::cout << "Memory initialized: " << memory.isInitialized << std::endl;  
    // std::cout << std::setprecision(1) << std::fixed;
    std::cout << "---------- Permanent Memory --------------------" << std::endl;  
    std::cout << "Bytes in use:     " << memory.current<< std::endl;
    std::cout << "Bytes left:       " << (memory.PermanentStorageSize - memory.current) << std::endl;
    std::cout << "Bytes total:      " << memory.PermanentStorageSize  << std::endl; 
    std::cout << std::endl; 
    std::cout << "---------- Transient Memory --------------------" << std::endl;  
    std::cout << "Bytes in use:     " << memory.transient_current << std::endl;
    std::cout << "Bytes left:       " << (memory.TransientStorageSize - memory.transient_current)  << std::endl;
    std::cout << "Bytes total:      " << memory.TransientStorageSize  << std::endl; 
    std::cout << "/------------------------------------------------" << std::endl;  
    std::cout << std::endl;  
} 

void initialize_memory(GameMemory &memory, uint64_t num_megabytes, uint64_t trans_megabytes = 1)
{
    memory = {};
    memory.PermanentStorageSize = Megabytes((uint64_t)num_megabytes);// 9 * 1024 * 1024;
    memory.PermanentStorage = malloc(memory.PermanentStorageSize);
    #ifdef MEM_DEBUG
    	if (memory.PermanentStorage)
    	{
            // std::cout << "Memory successfully mallocd." << std::endl;
    	} else {
    		std::cout << "ERROR: Failed to malloc memory." << std::endl;
    	}
    #endif
    
    memory.current = 0;
    memory.isInitialized = true;
    memset(memory.PermanentStorage,0,memory.PermanentStorageSize); 
    
    memory.TransientStorageSize = Megabytes((uint64_t)trans_megabytes);
    memory.TransientStorage = malloc(memory.TransientStorageSize);
    memory.transient_current = 0; 
    memset(memory.TransientStorage,0,memory.TransientStorageSize);  
}

#endif // _GAME_MEMORY_H_
