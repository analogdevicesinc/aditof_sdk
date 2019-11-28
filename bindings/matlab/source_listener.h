#ifndef __DEMO_SOURCE_LISTENER_HEADER__
#define __DEMO_SOURCE_LISTENER_HEADER__

#include "mwadaptorimaq.h"
#include "source_adaptor.h"

/**
 * Class SourceListener
 *
 * @brief:  Listens for changes in SelectedSource.
 *
 */
class SourceListener : public imaqkit::IEnginePropPostSetListener {

  public:
    //**************************************
    //* CONSTRUCTOR/DESTRUCTOR
    //**************************************
    /**
     * Constructor for SourceListener class.
     *
     * @param parent: Handle to the instance of the IAdaptor class
     *                that is the parent of this object.
     */
    SourceListener(SourceAdaptor *parent) : m_parent(parent){};

    // Destructor
    virtual ~SourceListener(void){};

    // *******************************************************************
    // METHODS FOR CONFIGURING AND UPDATING DEMO SOURCE INPUTS.
    // *******************************************************************
    /**
     * This is the method the engine calls when the value of the SelectedSource
     * property changes. notify() casts the new source value to the appropriate
     * type and then calls the SourceListener::applyValue() method to
     * configure the new source. To set up a listener for other properties, use
     * the DemoPropListener object.
     *
     * @param propertyInfo: Property information object.
     * @param newValue: New source ID value.
     *
     * @return void:
     */
    virtual void notify(imaqkit::IEnginePropInfo *propertyInfo, void *newValue);

  private:
    /**
     * Update and configure the specified property.
     *
     * @return void:
     */
    virtual void applyValue(void);

    /// The instance of the parent class that created this listener.
    SourceAdaptor *m_parent;

    /// The new source id requested by the user.
    int m_source;
};

#endif
