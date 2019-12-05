#ifndef __DEMO_PROP_LISTENER_HEADER__
#define __DEMO_PROP_LISTENER_HEADER__

#include "aditofimaq.h"
#include "mwadaptorimaq.h"
#include "source_adaptor.h"
#include <string>

/**
 * Class PropListener
 *
 * @brief:  Listens for changes in device-specific properties.
 *
 */
class PropListener : public imaqkit::IPropPostSetListener {

  public:
    // **************************************
    // CONSTRUCTOR/DESTRUCTOR
    // **************************************
    /**
     * Constructor for PropListener class.
     *
     * @param parent: Handle to the instance of the IAdaptor class
     *                that is the parent of this object.
     */
    PropListener(SourceAdaptor *parent) : m_parent(parent), m_lastIntValue(0) {}
    virtual ~PropListener(){};

    // *******************************************************************
    // METHODS FOR CONFIGURING AND UPDATING DEMO FEATURE PROPERTY VALUES.
    // *******************************************************************
    /**
     * This is the method the engine calls when a property value
     * changes. notify() casts the new property value to the appropriate
     * type and then calls the PropListener::applyValue() method to
     * configure the property.
     *
     * @param propertyInfo: The property information object.
     * @param newValue: The new value of the property.
     *
     * @return void:
     */
    virtual void notify(imaqkit::IPropInfo *propertyInfo, void *newValue);

  private:
    /**
     * applyValue: Find the property to update and configure it.
     *
     * @return void:
     */
    virtual void applyValue(void);

    /// The instance of the parent class that created this listener.
    SourceAdaptor *m_parent;

    /// Property Information object.
    imaqkit::IPropInfo *m_propInfo;

    /// The new value for integer properties.
    int16_t m_lastIntValue;

    /// The new value for double properties.
    double m_lastDoubleValue;

    /// The new value for string properties.
    std::string m_lastStrValue;
};

#endif
