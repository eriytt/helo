#ifndef SERIALIZE_HH
#define SERIALIZE_HH

#include <Ogre.h>

template <class S, class T>
struct SerializeHelper {
  static void serialize(S &stream, const unsigned short &s) {
    // std::cout << "Writing unsigned short '" << s << "' of size 2" << std::endl;
    stream.write(reinterpret_cast<const char *>(&s), 2);
  }

  static void serialize(S &stream, const unsigned int &i) {
    // std::cout << "Writing unsigned int '" << i << "' of size 4" << std::endl;
    stream.write(reinterpret_cast<const char *>(&i), 4);
  }
  
  static void serialize(S &stream, const Ogre::Real *r, int size) {
    // std::cout << "Writing Ogre::Real* '" << *r << ",...' of size " << size << std::endl;
    stream.write(reinterpret_cast<const char *>(r), size);
  }

  static void serialize(S &stream, const char *c, size_t size) {
    // std::cout << "Serializing char* '" << *c << "...' of size " << size << std::endl;
    stream.write(c, size);
  }
  
  static void serialize(S &stream, const std::string &str) {
    // std::cout << "Serializing std::string '" << str << "' of size " << str.length() << std::endl;
    serialize(stream, static_cast<uint16_t>(str.length()));
    serialize(stream, str.c_str(), str.length());
  }

  static void serialize(S &stream, const Ogre::Vector3 &v) {
    // std::cout << "Serializing Ogre::Vector3 "
    //           << v
    //           << " of size "
    //           << sizeof(Ogre::Real) * 3
    //           << std::endl;
    serialize(stream, v.ptr(), sizeof(Ogre::Real) * 3);
  }

  static void serialize(S &stream, const Ogre::Quaternion &q) {
    // std::cout << "Serializing Ogre::Quaternion "
    //           << q
    //           << " of size "
    //           << sizeof(Ogre::Real) * 4
    //           << std::endl;
    serialize(stream, q.ptr(), sizeof(Ogre::Real) * 4);
  }
};

template <class S>
class Serializer
{
private:
  S ostr;
  
public:
  template <typename T>
  Serializer<S> &operator<<(const T &obj) {
    SerializeHelper<S, T>::serialize(ostr, obj);
    return *this;
  }

  Serializer<S> &operator<< (std::ostream &(*pf)(std::ostream &os)) {
    ostr << pf;
    return *this;
  }

  S &getStream() {return ostr;}
};


#endif // SERIALIZE_HH
