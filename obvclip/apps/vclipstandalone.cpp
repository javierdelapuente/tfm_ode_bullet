#include <ostream>

#include "polytope.hpp"
#include "polytope_examples.hpp"
#include "vclip.hpp"
#include <chrono>


using namespace OB;

int main (int argc, char *argv[])
{
    std::vector<Point> points =  get_rbox_points("10000 s D3"); /// 1000 points in a 3d sphere

    std::shared_ptr<OB::ConvexPolytope> cv = std::make_shared<OB::ConvexPolytope>();
    cv->generate_from_vertices(points);
    assert(cv->euler_number_correct());

    OB::WorldConvexPolytope wcp0{"obj-1", cv};
    OB::Transform t0;
    t0 = OB::Translation(0, 5 ,0) * OB::AngleAxis(0.5*OB::PI, OB::Vector::UnitZ());
    wcp0.set_pose(t0);


    OB::WorldConvexPolytope wcp1{"obj-1", cv};
    OB::Transform t1;
    t1 = OB::Translation(-5, 5 ,0) * OB::AngleAxis(0.5*OB::PI, OB::Vector::UnitY());
    wcp1.set_pose(t1);

    Real margin = 5.0;
    VClipCache vclipCache;

    for (int i = 0; i < 1000000; ++i)
    {
        //std::chrono::high_resolution_clock::time_point tp1 = std::chrono::high_resolution_clock::now();
        VClip vclip{wcp0, wcp1};
        VClipResult result = vclip(vclipCache);
        //std::chrono::high_resolution_clock::time_point tp2 = std::chrono::high_resolution_clock::now();
        //auto int_us = std::chrono::duration_cast<std::chrono::microseconds>(tp2 - tp1);
        //auto integral_duration = static_cast<uint64_t>(int_us.count());
        //std::cout << "Result: " << result << " Time: " << integral_duration << "\n";

        //std::chrono::high_resolution_clock::time_point tp3 = std::chrono::high_resolution_clock::now();
        // {
        //     std::vector<ContactDataVClip> contactDataVclip = vclip.getContactData(margin);
        // }
        //std::chrono::high_resolution_clock::time_point tp4 = std::chrono::high_resolution_clock::now();
        //auto int_us_contact_data = std::chrono::duration_cast<std::chrono::microseconds>(tp4 - tp3);
        //auto integral_duration_contact_data = static_cast<uint64_t>(int_us_contact_data.count());
        //std::cout << "ContactData Time: " << integral_duration_contact_data << "\n";
    }


}
