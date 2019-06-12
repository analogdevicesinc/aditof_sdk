#ifndef STATUS_DEFINITIONS_H
#define STATUS_DEFINITIONS_H

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @enum Status
 * @brief Status of any operation that the TOF sdk performs.
 */
enum class Status {
    OK,               //!< Success
    BUSY,             //!< Device or resource is busy
    UNREACHABLE,      //!< Device or resource is unreachable
    INVALID_ARGUMENT, //!< Invalid arguments provided
    GENERIC_ERROR     //!< An error occured but there are no details available.
};

} // namespace aditof

#endif // STATUS_DEFINITIONS_H
