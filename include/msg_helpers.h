
#ifndef MSG_HELPERS_HARE
#define MSG_HELPERS_HARE

struct __attribute__((packed)) PointCloudInfo {
    double timestamp;
    uint8_t index;
};


std::vector<uint8_t> serialize(const PointCloudInfo& data) {
    // Create a byte vector with enough space
    std::vector<uint8_t> byteArray(sizeof(PointCloudInfo));
    
    // Copy data from the struct to the byte array
    std::memcpy(byteArray.data(), &data, sizeof(PointCloudInfo));

    return byteArray;
}

PointCloudInfo deserialize(const uint8_t* data, size_t length) {
    // Ensure the byte array is the correct size
    if (length != sizeof(PointCloudInfo)) {
        throw std::runtime_error("Byte array size does not match struct size.");
    }

    PointCloudInfo result;
    
    // Copy data from the byte array to the struct
    std::memcpy(&result, data, sizeof(PointCloudInfo));
    
    return result;
}

#endif // MSG_HELPERS_HARE
