#include "objectTextDisplay.hpp"

using namespace OB;

Ogre::Overlay * ObjectTextDisplay::g_pOverlay = nullptr;

void ObjectTextDisplay::update()  {
    if (!m_enabled) return;

    const Ogre::AxisAlignedBox& bbox = m_p->getWorldBoundingBox(true);
    Ogre::Matrix4 mat = m_c->getProjectionMatrix() * m_c->getViewMatrix();

    // We want to put the text point in the center of the top of the AABB Box.
    Ogre::Vector3 topcenter = bbox.getCenter();
//    topcenter.z += bbox.getHalfSize().z;    // in world coordinates, z is world up
    topcenter = mat * topcenter;
    bool behind = (topcenter.z < 0.0);      // in view and clip coordinates, z is towards the screen

    if (behind) {
        // Don't show text for objects behind the camera.
        m_pContainer->setPosition(-1000, -1000);
    }
    else {
        // After multiplying the topcenter vector by the projection matrix and view matrix,
        // we have the coordinates in "clip space", i.e., screen coordinates clipped to (-1, 1) (where -1 is left and top, +1 is right and bottom)
        // since setPosition requires coordinates in (0, 1), we need some more math.
        m_pContainer->setPosition(topcenter.x / 2 + 0.5, -topcenter.y / 2 + 0.5);

        // Size is relative to screen size being 1.0 by 1.0 (not pixels size)
        //m_pContainer->setDimensions(0.1f, 0.1f);
        m_pContainer->setDimensions(0.01f, 0.01f);
        //m_pContainer->setMaterialName("RedTransparent");
    }

}
