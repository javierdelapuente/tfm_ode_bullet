#ifndef OBUGRE_OBJECTTEXTDISPLAY_HPP
#define OBUGRE_OBJECTTEXTDISPLAY_HPP

#include <Ogre.h>
#include <OgreFont.h>
#include <OgreFontManager.h>
#include <OgreOverlay.h>
#include <OgreOverlayElement.h>
#include <OgreOverlayManager.h>
#include <OgreOverlayContainer.h>

#include <oblog.hpp>

namespace OB
{

    // From the Ogre forums.  This class displays text above or next to on screen entities.
    class ObjectTextDisplay {

    public:
        static Ogre::Overlay* g_pOverlay;

        ObjectTextDisplay(const Ogre::MovableObject* p, Ogre::Camera* c) {
            m_p = p;
            m_c = c;
            m_enabled = false;
            m_text = "";

            // create an overlay that we can use for later
            if(!g_pOverlay)
            {
                g_pOverlay = Ogre::OverlayManager::getSingleton().create("floatingTextOverlay");
                g_pOverlay->show();
            }

            // be careful with this c-string
            char buf[30];
            sprintf(buf, "c_%s", p->getName().c_str());
            m_elementName = buf;
            m_pContainer = static_cast<Ogre::OverlayContainer*>
                (Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", buf));

            g_pOverlay->add2D(m_pContainer);

            sprintf(buf, "ct_%s", p->getName().c_str());
            m_elementTextName = buf;
            m_pText = Ogre::OverlayManager::getSingleton().createOverlayElement("TextArea", buf);
            m_pText->setDimensions(1.0, 1.0);
            m_pText->setMetricsMode(Ogre::GMM_PIXELS);
            m_pText->setPosition(0, 0);

            m_pText->setParameter("font_name", "SdkTrays/Caption");
            //m_pText->setParameter("char_height", "8");
            m_pText->setParameter("char_height", "10");
            m_pText->setParameter("horz_align", "center");
            m_pText->setColour(Ogre::ColourValue(0.0, 0.0, 0.0));
            //m_pText->setColour(Ogre::ColourValue(1.0, 1.0, 1.0));

            m_pContainer->addChild(m_pText);
            m_pContainer->setEnabled(false);


        }

        virtual ~ObjectTextDisplay() {

            // overlay cleanup -- Ogre would clean this up at app exit but if your app
            // tends to create and delete these objects often it's a good idea to do it here.

            //JP THIS CRASHES
            Ogre::OverlayManager *overlayManager = Ogre::OverlayManager::getSingletonPtr();
            // m_pContainer->removeChild(m_elementTextName);
            // g_pOverlay->remove2D(m_pContainer);
            // overlayManager->destroyOverlayElement(m_pText);
            // overlayManager->destroyOverlayElement(m_pContainer);
        }

        void enable(bool enable) {
            m_enabled = enable;
            if (enable)
            {
                m_pContainer->show();
            }
            else
            {
                m_pContainer->hide();
            }
        }

        void setText(const Ogre::String& text) {
            m_text = text;
            m_pText->setCaption(m_text);
        }



        void update();

    protected:
        const Ogre::MovableObject* m_p;
        Ogre::Camera* m_c;
        bool m_enabled;
        Ogre::OverlayElement* m_pText;
        Ogre::String m_text;
        Ogre::String m_elementName;
        Ogre::String m_elementTextName;
        Ogre::OverlayContainer* m_pContainer;
    };
}

#endif
