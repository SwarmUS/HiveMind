#ifndef __CIRCULAR_BUFF_H_
#define __CIRCULAR_BUFF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 *@brief A structure to manage a circular buffer
 **/
typedef struct {
    uint8_t* data;
    uint16_t size;
    uint16_t readPos;
    uint16_t writePos;
    bool isFull;
} CircularBuff;

/**
 *@brief The return type of some function associated with the circular buffer to notify if the
 *operation was successfull or not*/
typedef enum {
    /** The operation was successfull*/
    CircularBuff_Ret_Ok,

    /** The operation was not successfull, a null pointer was passed or the circular buff is not
       initialised*/
    CircularBuff_Ret_NullErr,

    /** The operation was not successfull, the buffer doesn't have enough space*/
    CircularBuff_Ret_SpaceErr,

    /** The operation was not successfull, the buffer is empty on a read operation*/
    CircularBuff_Ret_EmptyErr,
} CircularBuffRet;

/**
 *
 * @brief Initialise the circular buffer struc
 *
 * @param [in] circularBuff the struct to initialise
 *
 * @param [in] buffer the buffer that will be used to store data, needs to live for the whole
 *lifetime of the circular buffer
 *
 * @param [in] size the size of the data buffer provided
 *
 * @return false if the operation was not successfull (if a null pointer is
 *provided), otherwise true
 **/
bool CircularBuff_init(CircularBuff* circularBuff, uint8_t* buffer, uint16_t size);

/**
 * @brief Gets the available size of the circularBuffer
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @return the size currently availaible in the circular buffer, returns 0 if the pointer is null or
 *the buffer is no initialized
 **/
uint16_t CircularBuff_getFreeSize(const CircularBuff* circularBuff);

/**
 * @brief Gets the current length of the data in the buffer
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @return the data length currently in the circular buffer, returns 0 if the pointer is
 *null or the buffer is not initialized
 **/
uint16_t CircularBuff_getLength(const CircularBuff* circularBuff);

/**
 * @brief Checks if the buffer is full or not
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @return true if there is no space available in the buffer or if the circularBuffer is null or
 *initialized, otherwise false
 **/
bool CircularBuff_isFull(const CircularBuff* circularBuff);

/**
 * @brief Add a byte to the circular buffer
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @param [in] data the data to insert in the buffer
 *
 **/
CircularBuffRet CircularBuff_putc(CircularBuff* circularBuff, uint8_t data);

/**
 * @brief Add multiple bytes to the circular buffer
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @param [in] data pointer to the array to insert in the buffer
 *
 * @param [in] length the size of the data to insert
 *
 **/
CircularBuffRet CircularBuff_put(CircularBuff* circularBuff, const uint8_t* data, uint16_t length);

/**
 * @brief Read a byte from the circular buffer
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @param [out] data pointer where the value will be store
 *
 **/
CircularBuffRet CircularBuff_readc(CircularBuff* circularBuff, uint8_t* data);

/**
 * @brief Read multiple bytes from the circular buffer
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @param [out] data pointer where the values will be store
 *
 * @param [in] length to read
 *
 * @return the actual number of bytes read from the buffer. Returns 0 if a pointer is null or the
 *circularBuff is not initialized
 **/
uint16_t CircularBuff_read(CircularBuff* circularBuff, uint8_t* data, uint16_t length);

/**
 * @brief Clears the buffer from data
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @return true if the operation was successfull, false if a null pointer or the circularBuff was
 *not initialized
 **/
bool CircularBufer_clear(CircularBuff* circularBuff);

#ifdef __cplusplus
}
#endif

#endif // __CIRCULAR_BUFF_H_
