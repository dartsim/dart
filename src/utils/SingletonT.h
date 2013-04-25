#ifndef SINGLETONT_H
#define SINGLETONT_H

template <class T>
class SingletonT
{
  /// \brief Get an instance of the singleton
  public: static T *Instance()
          {
            return &getInstance();
          }

  /// \brief Constructor
  protected: SingletonT() {}

  /// \brief Destructor
  protected: virtual ~SingletonT() {}

  /// \brief Creates and returns a reference to the unique (static) instance
  private: static T &getInstance()
           {
             static T t;
             return static_cast<T &>(t);
           }

  /// \brief A reference to the unique instance
  private: static T &myself;
};

/// \brief Initialization of the singleton instance.
template <class T>
T &SingletonT<T>::myself = SingletonT<T>::getInstance();

#endif

