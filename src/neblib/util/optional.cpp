#include "optional.hpp"

template <typename T>
Optional<T>::Optional() : m_hasValue(false) {}

template <typename T>
Optional<T>::Optional(const T& val) : m_hasValue(true), m_value(val) {}

template <typename T>
Optional<T>::Optional(T&& val) : m_hasValue(true), m_value(std::move(val)) {}

template <typename T>
Optional<T>::~Optional()
{
    reset();
}

template <typename T>
void Optional<T>::setValue(const T &val)
{
    m_hasValue = true;
    m_value = val;
}

template <typename T>
void Optional<T>::setValue(T &&val)
{
    m_hasValue = true;
    m_value = std::move(val);
}

template <typename T>
bool Optional<T>::hasValue() const
{
    return m_hasValue;
}

template <typename T>
const T &Optional<T>::value() const
{
    if (!m_hasValue) throw std::runtime_error("No value");
    else return m_value;
}

template <typename T>
T &Optional<T>::value()
{
    if (!m_hasValue) throw std::runtime_error("No value");
    else return m_value;
}

template <typename T>
void Optional<T>::reset()
{
    if (m_hasValue) 
    {
        m_hasValue = false;
        m_value.~T();
    }
}

template <class T, class U>
inline constexpr bool operator==(const Optional<T> &lhs, const Optional<U> &rhs)
{
    if (lhs.hasValue() == rhs.hasValue()) 
    {
        if (lhs.hasValue()) return lhs.value() == rhs.value();
        else return true;
    }
    else return false;
}