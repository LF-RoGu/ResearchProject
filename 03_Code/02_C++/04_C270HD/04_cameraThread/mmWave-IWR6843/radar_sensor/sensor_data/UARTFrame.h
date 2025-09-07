#ifndef UARTFRAME_H
#define UARTFRAME_H

#include "lib/SensorDataStr.h"
#include "lib/EndianUtils.h"
#include <vector>
#include <cstdint>
#include <iostream>

/**
 * @class UART_frame
 * @brief Base class for handling UART frame operations such as parsing and endian conversion.
 */
class UART_frame {
protected:
    std::vector<uint8_t> UARTFrame_vec;  ///< Vector to hold raw UART frame data.

public:
    /**
     * Default constructor for UART_frame.
     */
    UART_frame();

};

/**
 * @class Frame_header
 * @brief Derived class for parsing and handling frame headers.
 */
class Frame_header : public UART_frame {
private:
    FrameHeaderData FrameHeader_str;  ///< Struct to store parsed frame header data.

public:
    bool frameValid;  ///< Flag to indicate if the frame header is valid.
    bool isValid() const { return frameValid; }
    Frame_header();
    Frame_header(std::vector<uint8_t>& data);

    /**
     * Parses the entire frame header from raw data, interpreting each multi-byte field
     * in little-endian format. The magic word is used to validate the start of the frame.
     * Input:
     *  - data: Vector of raw data containing the frame.
     * Output:
     *  - FrameHeaderData: Parsed frame header data.
     */
    void parseFrameHeader(std::vector<uint8_t>& data);

    // Setters


    /**
     * Sets the Magic Word in the frame header.
     * Input:
     *  - var: The version value to set.
     */
    void setMagicWord(uint64_t var);
    /**
     * Sets the version field in the frame header.
     * Input:
     *  - var: The version value to set.
     */
    void setVersion(uint32_t var);

    /**
     * Sets the packet length field in the frame header.
     * Input:
     *  - var: The packet length value to set.
     */
    void setPacketLength(uint32_t var);

    /**
     * Sets the platform field in the frame header.
     * Input:
     *  - var: The platform value to set.
     */
    void setPlatform(uint32_t var);

    /**
     * Sets the frame number field in the frame header.
     * Input:
     *  - var: The frame number value to set.
     */
    void setFrameNumber(uint32_t var);

    /**
     * Sets the timestamp field in the frame header.
     * Input:
     *  - var: The timestamp value to set.
     */
    void setTime(uint32_t var);

    /**
     * Sets the number of detected objects in the frame header.
     * Input:
     *  - var: The number of detected objects to set.
     */
    void setNumObjDetecter(uint32_t var);

    /**
     * Sets the number of TLVs in the frame header.
     * Input:
     *  - var: The number of TLVs to set.
     */
    void setNumTLV(uint32_t var);

    /**
     * Sets the subframe number field in the frame header.
     * Input:
     *  - var: The subframe number value to set.
     */
    void setSubframeNum(uint32_t var);


    // Getters
    /**
     * Gets the MagicWord from the frame header.
     * Input:
     *  - None.
     * Output:
     *  - uint64_t: The MagicWord.
     */
    uint64_t getMagicWord() const;
    /**
     * Gets the version field from the frame header.
     * Input:
     *  - None.
     * Output:
     *  - uint32_t: The version value.
     */
    uint32_t getVersion() const;

    /**
     * Gets the packet length field from the frame header.
     * Input:
     *  - None.
     * Output:
     *  - uint32_t: The packet length value.
     */
    uint32_t getPacketLength() const;

    /**
     * Gets the platform field from the frame header.
     * Input:
     *  - None.
     * Output:
     *  - uint32_t: The platform value.
     */
    uint32_t getPlatform() const;

    /**
     * Gets the frame number field from the frame header.
     * Input:
     *  - None.
     * Output:
     *  - uint32_t: The frame number value.
     */
    uint32_t getFrameNumber() const;

    /**
     * Gets the timestamp field from the frame header.
     * Input:
     *  - None.
     * Output:
     *  - uint32_t: The timestamp value.
     */
    uint32_t getTime() const;

    /**
     * Gets the number of detected objects from the frame header.
     * Input:
     *  - None.
     * Output:
     *  - uint32_t: The number of detected objects.
     */
    uint32_t getNumObjDetecter() const;

    /**
     * Gets the number of TLVs from the frame header.
     * Input:
     *  - None.
     * Output:
     *  - uint32_t: The number of TLVs.
     */
    uint32_t getNumTLV() const;

    /**
     * Gets the subframe number field from the frame header.
     * Input:
     *  - None.
     * Output:
     *  - uint32_t: The subframe number value.
     */
    uint32_t getSubframeNum() const;
};

/**
 * @class TLV_header
 * @brief Derived class for parsing TLV (Type-Length-Value) headers.
 */
class TLV_frame{
private:
    TLVHeaderData TLVHeaderData_str;
    TLVPayloadData TLVPayloadData_str;
public:
    /**
     * Default constructor for TLV_header.
     */
    TLV_frame();

    TLV_frame(std::vector<uint8_t>& data, uint32_t numDetectedObj_var);
    
    /**
     * Parses the TLV header from raw data.
     * Input:
     *  - data: Pointer to the raw data buffer containing TLVs.
     *  - offset: Reference to the current offset in the data buffer.
     * Output:
     *  - void: Updates TLVHeaderData_str with the parsed data.
     */
    TLVHeaderData parseTLVHeader(std::vector<uint8_t>& data);
    /**
     * Parses the TLV payload from raw data based on the header information.
     *
     * Input:
     *  - data: Reference to the vector of raw data containing the payload.
     *  - offset: Reference to the current offset in the data buffer.
     *  - header: Constant reference to the TLV header data structure.
     *
     * Output:
     *  - void: Updates the payload data based on the parsed information.
     */
    TLVPayloadData parseTLVPayload(std::vector<uint8_t>& data, TLVHeaderData TLVHeaderData_var, uint32_t numDetectedObj_var);

    TLVHeaderData getTLVFrameHeaderData();
    TLVPayloadData getTLVFramePayloadData();
};

class TLV_header : public TLV_frame
{
private:
    TLVHeaderData TLVHeaderData_str;
public:
    /**
     * Default constructor for TLV_header.
     */
    TLV_header();

    /**
     * Sets the Type identifier.
     * @param type: The type value to set.
     */
    void setType(uint32_t var);

    /**
     * Sets the Length of the payload.
     * @param length: The length value to set.
     */
    void setLength(uint32_t var);

    /**
     * Returns the Type identifier.
     * @return uint32_t Type value.
     */
    uint32_t getType() const;

    /**
     * Returns the Length of the payload.
     * @return uint32_t Length value.
     */
    uint32_t getLength() const;
};

class TLV_payload : public TLV_frame
{
private:
    // Updated parsePayload function to handle different TLV types
    std::vector<TLVPayloadData> TLVPayloadData_vect;
public:
    /**
     * Default constructor for TLV_payload.
     */
    TLV_payload();
    /**
     * Constructor that takes raw data and number of detected objects,
     * and calls the base class constructor for parsing.
     */
    TLV_payload(std::vector<uint8_t>& data, uint32_t numDetectedObj_var, uint32_t numTLVs_var);

    // Setters for vectors
    void setTLVPayloadData(TLVPayloadData TLVPayloadData_var);

    // Getters for vectors
    std::vector<TLVPayloadData> getTLVPayloadData();

};

#endif // UARTFRAME_H
