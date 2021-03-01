#ifndef OBVCLIP_CONTACTS_HPP
#define OBVCLIP_CONTACTS_HPP

#include "vclip.hpp"

namespace OB
{

    // in world coordinates.
    // like ODE, """The convention is that if body 1 is moved along
    // the normal vector by a distance depth (or equivalently if body 2
    // is moved the same distance in the opposite direction) then the
    // contact depth will be reduced to zero. This means that the normal
    // vector points "in" to body 1."""
    // distance = -depth
    ContactDataVClip get_simple_contact(WorldConvexPolytope& wcp1,
                                        WorldConvexPolytope& wcp2,
                                        VClipResult &lastresult);

    std::vector<ContactDataVClip> get_contacts_envelope(WorldConvexPolytope& wcp1,
                                                        WorldConvexPolytope& wcp2,
                                                        VClipResult &lastresult,
                                                        Real envelope = 0.04);

}

#endif
