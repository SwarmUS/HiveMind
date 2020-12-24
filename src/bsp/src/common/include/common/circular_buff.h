#ifndef __CIRCULAR_BUFF_H_
#define __CIRCULAR_BUFF_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

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
 *@brief A structure to manage a circular buffer, you should never need to touch the data here, only
 *use the associated function
 **/
typedef struct {
    uint8_t* m_data;
    uint16_t m_size;
    uint16_t m_readPos;
    uint16_t m_writePos;
    bool m_isFull;
} CircularBuff;

/**
 *@brief A structure to manage a buffer with a certain size, mainly to get data without copy
 *
 **/
typedef struct {
    /** A pointer to the data to access */
    const uint8_t* data;
    /** The length of the available data */
    const uint16_t length;
    /** The status of the operation */
    const CircularBuffRet status;
} ZeroCopyBuff;

/**
 *
 * @brief Initialise the circular buffer struct
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
 * @brief Read a byte from the circular buffer, copy the value to the provided data buffer
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @param [out] data pointer where the value will be store
 *
 **/
CircularBuffRet CircularBuff_getc(CircularBuff* circularBuff, uint8_t* data);

/**
 * @brief Read multiple bytes from the circular buffer, copy the value to the provided data buffer
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
uint16_t CircularBuff_get(CircularBuff* circularBuff, uint8_t* data, uint16_t length);

/**
 * @brief Read multiple bytes from the circular buffer without copy, you shouldn't modify the data
 *of the buff. Mostly for DMA or ISR. Does not make the read pointer go forward to avoir write
 *during reads.
 *
 *@b Warning, since this is a ring buffer, it cannot loop using without copy, so you may need to
 *call it twice to obtain all the data. First call will read all the data until the end of the
 *buffer, then set the read pointer at the start. The second call with get the remaining data at the
 *beginning of the buffer.
 *
 *@b Warning The zero copy function does not increment the read pointer, since that would allow
 *overwrite while you are using the read pointer. Use the CircularBuff_advance function when you are
 *done with the read.
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @param [in] length to read
 *
 * @return A Buff struct, allowing you to access the data and get the return value
 **/
ZeroCopyBuff CircularBuff_getZeroCopy(CircularBuff* circularBuff, uint16_t length);

/**
 * @brief Allows to go forward without reading data, mostly used with CircularBuff_getZeroCopy when
 *you are done with the data.
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @param [in] length go forward a number of bytes
 *
 * @return the actual number of bytes advanced in the buffer. Returns 0 if a pointer is null or the
 *circularBuff is not initialized
 **/
uint16_t CircularBuff_advance(CircularBuff* circularBuff, uint16_t length);

/**
 * @brief Clears the buffer from data
 *
 * @param [in] circularBuff the buffer to operate on
 *
 * @return true if the operation was successfull, false if a null pointer or the circularBuff was
 *not initialized
 **/
bool CircularBuff_clear(CircularBuff* circularBuff);

#ifdef __cplusplus
}
#endif

#endif // __CIRCULAR_BUFF_H_
