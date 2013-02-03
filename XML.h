
namespace XMLUtils {

  class XMLError : public std::runtime_error
  {
  public:
    XMLError(const std::string &reason) : std::runtime_error(reason) {}
  };

  template <typename T>
  T GetAttrValue(const TiXmlAttribute *attr)
  {
    throw XMLError("Not implemented for this type");
  }

  template <>
  int GetAttrValue(const TiXmlAttribute *attr)
  {
    int ret = 0;
    if (attr->QueryIntValue(&ret) !=  TIXML_SUCCESS)
      throw XMLError("Cannot convert value '" + attr->ValueStr() + "' to int.");
    return ret;
  }

  template <>
  double GetAttrValue(const TiXmlAttribute *attr)
  {
    double ret = 0.0;
    if (attr->QueryDoubleValue(&ret) !=  TIXML_SUCCESS)
      throw XMLError("Cannot convert value '" + attr->ValueStr() + "' to double.");
    return ret;
  }

  template <>
  float GetAttrValue(const TiXmlAttribute *attr)
  {
    double ret;
    try
      {
        ret = GetAttrValue<double>(attr);
      }
    catch (XMLError e)
      {
        throw XMLError("Cannot convert value '" + attr->ValueStr() + "' to float.");        
      }
    return static_cast<float>(ret);
  }

  template <>
  std::string GetAttrValue(const TiXmlAttribute *attr)
  {
    return attr->ValueStr();
  }

  template <typename T>
  T GetAttribute(const std::string attrname, const TiXmlNode* node)
  {
    const TiXmlElement *element = node->ToElement();
    if (not element)
      throw XMLError("Node " + node->ValueStr() + "is not an element");

    const TiXmlAttribute *attr = element->FirstAttribute();
    if (not attr)
      goto nosuchattribute;
    
    do 
      {
        if (attrname == attr->Name())
          try
            {
              return GetAttrValue<T>(attr);
            }
          catch (XMLError e)
            {
              throw XMLError("Node '" + node->ValueStr() + "', attribute '" + attrname + "': "
                             + e.what());
            }
      }
    while ((attr = attr->Next()));

nosuchattribute:
    throw XMLError("Element " + element->ValueStr() + " has no such attribute");
  }

  template <typename T>
  T GetVectorParam(const std::string paramname, TiXmlNode* node)
  {
     TiXmlNode *child = node->IterateChildren(paramname, NULL);
     if (not child)
       throw XMLError("Node " + node->ValueStr() + " has no '" + paramname +"' child");

     T res;
     res.x = GetAttribute<float>("x", child);
     res.y = GetAttribute<float>("y", child);
     res.z = GetAttribute<float>("z", child);

     return res;
  }

  unsigned int GetNumChildren(const TiXmlNode *parent, const std::string &name = std::string(""))
  {
    const TiXmlNode *node = NULL;
    unsigned int count = 0;

    if (name != "")
      while((node = parent->IterateChildren(name, node)))
        ++count;
    else
      while((node = parent->IterateChildren(node)))
        ++count;
    return count;
  }

}
