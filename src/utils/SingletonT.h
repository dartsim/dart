#ifndef SINGLETONT_H
#define SINGLETONT_H

template <class T>
class SingletonT
{
public:
    /// \brief Get an instance of the singleton
    static T *Instance()
    {
        return &getInstance();
    }

protected:
    /// \brief Constructor
    SingletonT() {}

    /// \brief Destructor
    virtual ~SingletonT() {}

private:
    /// \brief Creates and returns a reference to the unique (static) instance
    static T &getInstance()
    {
        static T t;
        return static_cast<T &>(t);
    }

    /// \brief A reference to the unique instance
    static T &myself;
};

/// \brief Initialization of the singleton instance.
template <class T>
T &SingletonT<T>::myself = SingletonT<T>::getInstance();

#endif

