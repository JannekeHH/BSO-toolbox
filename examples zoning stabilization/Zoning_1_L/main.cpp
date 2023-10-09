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
#include <bso/spatial_design/Zoning.hpp>
#include <bso/structural_design/sd_model.hpp>
//#include <bso/Building_Physics/BP_Simulation.hpp>
#include <bso/visualization/visualization.hpp>
//#include <bso/Performance_Indexing.hpp>
#include <bso/grammar/sd_grammars/Grammar_zoning.hpp>

int main(int argc, char* argv[])
{
    out("Heya, welcome to my program", true, false);
    bso::spatial_design::MS_Building MS("MS_Input.txt");out("Initialised the MS model",true, true);
    bso::spatial_design::MS_Conformal CF(MS, &(bso::Grammar::grammar_zoning));out("Initialised the CF model",true, true);

    CF.make_conformal();out("Made the CF model conformal",true, true);

    bso::spatial_design::Zoning::Zoned_Design Zoned(&CF);
    out("Initialised the zoning model", true, true);
    Zoned.make_zoning();
    out("finished zoning",true,true);



    out("Commencing visualization....",false, false);
    bso::visualization::init_visualization(argc, argv);

    bso::visualization::visualise(MS);
    //bso::visualization::visualise(MS,"space_type");
    //bso::visualization::visualise(MS,"surface_type");
    //bso::visualization::visualise(CF,"rectangles");
    bso::visualization::visualise(CF,"cuboids");

    // vis Zoned Designs
    for (unsigned int i = 0; i < Zoned.get_designs().size(); i++)
    {
        bso::visualization::visualise(CF,"zones", i);
    }

    // SD-analysis unzoned design
    Zoned.reset_SD_model();
    Zoned.prepare_unzoned_SD_model();
    bso::structural_design::sd_model SD_Building(CF);
    SD_Building.analyse();
    bso::structural_design::SD_Building_Results sd_results = SD_Building.get_results();
    bso::SD_compliance_indexing(sd_results);
    std::cout << std::endl << "Total compliance in the unzoned design: " << sd_results.m_total_compliance
    << std::endl << "Structural volume: " << sd_results.m_struct_volume << std::endl;
    Zoned.add_unzoned_compliance(sd_results.m_total_compliance);
    bso::visualization::visualise(SD_Building, 1);

    // SD-analysis zoned designs
	std::vector<double> m_compliance;
	std::vector<double> m_volume;
    for (unsigned int i = 0; i < Zoned.get_designs().size(); i++)
    {
        Zoned.reset_SD_model();
        Zoned.prepare_zoned_SD_model(i);
        bso::structural_design::sd_model SD_Building(CF);
        SD_Building.analyse();
        bso::structural_design::SD_Building_Results sd_results = SD_Building.get_results();
        bso::SD_compliance_indexing(sd_results);
        std::cout << "Total compliance in zoned design " << i + 1 << ": "
        << sd_results.m_total_compliance << std::endl << "Structural volume: " << sd_results.m_struct_volume << std::endl;
        Zoned.add_compliance(sd_results.m_total_compliance, i);
        bso::visualization::visualise(SD_Building, 1);
		m_compliance.push_back(sd_results.m_total_compliance);
		m_volume.push_back(sd_results.m_struct_volume);
    }
	std::cout << std::endl << "Compliances:" << std::endl;
	for (unsigned int i = 0 ; i < m_compliance.size(); i++)
	{
		std::cout << m_compliance[i] << std::endl;
	}
	std::cout << std::endl << "Volumes:" << std::endl;
	for (unsigned int i = 0 ; i < m_volume.size(); i++)
	{
		std::cout << m_volume[i] << std::endl;
	}


    std::cout << "Lowest total compliance (" << Zoned.get_min_compliance().first << ") in zoned design " <<
    Zoned.get_min_compliance().second + 1 << std::endl;
    Zoned.reset_SD_model();
    Zoned.prepare_zoned_SD_model(Zoned.get_min_compliance().second);
    bso::structural_design::sd_model SD_Zoned_Building(CF);
    SD_Zoned_Building.analyse();
    bso::structural_design::SD_Building_Results sd_zoned_results =
    SD_Zoned_Building.get_results();
    bso::SD_compliance_indexing(sd_zoned_results);
    bso::visualization::visualise(SD_Zoned_Building, 4);

    // vis SD_Building
    //bso::visualization::visualise(SD_Building, 2);
    //bso::visualization::visualise(SD_Building, 1);
    //bso::visualization::visualise(SD_Building, 4);

    out("Finished visualization",true, true);

    bso::visualization::end_visualization();
    return 0;
}
