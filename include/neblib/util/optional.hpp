#pragma once

#include <utility>
#include <stdexcept>

template <typename T>
class Optional {
    /**
     * @brief Implements functionality of the std::optional class under <optional> from c++17
     */
private:
    bool m_hasValue;
    T m_value;

public:
    /**
     * @brief Constructor with no value
     */
    Optional();

    /**
     * @brief Constructor with value -- Copy constructor
     * 
     * @param val the value to be passed to the object
     */
    Optional(const T& val);

    /**
     * @brief Constructor with value -- Move constructor
     * 
     * @param val the value to be passed to the object
     */
    Optional(T&& val);

    /**
     * @brief Sets the value of the Optional object
     * 
     * @param val the value to be passed to the object
     */
    void setValue(const T& val);

    /**
     * @brief Sets the value of the Optional object
     * 
     * @param val the value to be passed to the object
     */
    void setValue(T&& val);

    /**
     * @brief Returns whether or not the Optional has a value
     * 
     * @return true if optional has a value, false otherwise
     */
    bool hasValue() const;

    /**
     * @brief Gets the value of the Optional
     */
    const T& value() const;

    /**
     * @brief Gets the value of the Optional
     */
    T& value();

    /**
     * @brief Resets the Optional to having no value
     */
    void reset();
};

/**
 * @todo
 * More operators
 */

template <class T, class U>
constexpr bool operator==(const Optional<T>& lhs, const Optional<U>& rhs);