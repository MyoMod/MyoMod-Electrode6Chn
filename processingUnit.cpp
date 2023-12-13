/**
 * @file processingUnit.cpp
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 13.12.2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "processingUnit.h"
#include "math.h"
#include "assert.h"
#include "string.h"

// Defines

// Global Variables

// functions

/**
 * @brief This function will process the data in the inBuffer and store the result in the outBuffer
 * 
 * @param inBuffer      Pointer to the buffer containing the data to be processed
 * @param outBuffer     Pointer to the buffer where the result of the processing shall be stored
 * @param length        The length of the buffers in bytes
 */
void processData(void* inBuffer, void* outBuffer, uint32_t length)
{
    assert(inBuffer != NULL);
    assert(outBuffer != NULL);
    assert(length > 0);

    // copy the data from the inBuffer to the outBuffer
    memcpy(outBuffer, inBuffer, length);
}