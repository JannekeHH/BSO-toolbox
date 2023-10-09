#include <iostream>
#include <ctime>

static double end = clock();
static double begin = end;

template<class T>
void out(T t, bool e, bool i){
    std::cout << t;
    end = clock();
    if (i) std::cout << " (" << 1000*(end-begin)/CLOCKS_PER_SEC << " ms)";
    if (e) std::cout << std::endl;
    begin = end;
} // out()

#include <bso/spatial_design/ms_building.hpp>
#include <bso/spatial_design/cf_building.hpp>
#include <bso/spatial_design/zoning.hpp>
#include <bso/structural_design/sd_model.hpp>
#include <bso/structural_design/Stabilization/Stabilize.hpp>
#include <bso/visualization/visualization.hpp>
//#include <bso/Performance_Indexing.hpp>
#include <bso/grammar/sd_grammars/Grammar_stabilize.hpp>

int main(int argc, char* argv[])
{
    bso::Spatial_Design::MS_Building MS("MS_Input.txt");out("Initialised the MS model",true, true);
    bso::Spatial_Design::MS_Conformal CF(MS, &(bso::Grammar::grammar_stabilize));out("Initialised the CF model",true, true);
    CF.make_conformal();out("Made the CF model conformal",true, true);

    out("Commencing Visualisation...",false, false); std::cout << std::endl;
    bso::Visualisation::init_visualisation(argc, argv);
    bso::Visualisation::visualise(MS);
    bso::Visualisation::visualise(CF,"rectangles");

    out("Commencing SD-Analysis...",false, false);
    bso::Structural_Design::SD_Analysis SD_Building(CF);

    bso::Visualisation::end_visualisation();out("Finished visualisation",true, true);

    return 0;
}
