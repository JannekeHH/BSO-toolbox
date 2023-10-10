#ifndef STABILIZE_HPP
#define STABILIZE_HPP
#include <vector>
#include <algorithm>
#include <iostream>
#include <map>
#include <bso/structural_design/SD_Analysis.hpp> //nog vervangen, fea?
#include <bso/structural_design/component/point.hpp>
#include <bso/structural_design/stabilization/grid.hpp>
#include <bso/spatial_design/zoning.hpp>
#include <bso/grammar/read_stabilize_settings.hpp>

namespace bso { namespace structural_design { namespace stabilization
{
    struct zone
    {
        spatial_design::zoning::zone* zone;
		std::vector<Components::Point*> points;
		grid* grid;
        std::map<Components::Point*, std::vector<unsigned int> > points_grid;
		std::map<std::vector<unsigned int>, Components::Point*> grid_points;
		unsigned int x_size;
		unsigned int y_size;
		unsigned int z_size;
		bool primary = false;
		int floor;
		int floor_above;
		bool north_floating = false;
		bool east_floating = false;
		bool south_floating = false;
		bool west_floating = false;

		bool stable = true;
		bool stable_bottom = true;
		bool stable_top = true;
    }; //zone

	class stabilize
    {
    private:
        SD_Analysis_Vars* m_SD;
        spatial_design::MS_Conformal* m_CF;
		spatial_design::zoning::zoned_Design* zoned;
		std::vector<zone> zones;
        grid* m_GR;
        std::vector<std::vector<std::vector<coord*>>> grid;
		std::vector<Components::Point*> m_points;
        std::map<Components::Point*, std::vector<unsigned int> > points_grid;
		std::map<std::vector<unsigned int>, Components::Point*> grid_points;
		unsigned int x_size;
		unsigned int y_size;
		unsigned int z_size;
		std::vector<int> floors;
		std::vector<int> floor_coords;
		std::map<int, unsigned int> floor_grid;
		std::map<unsigned int, int> grid_floor;
		std::map<Components::Point*, int> point_floor;
		std::map<int, Components::Point*> floor_point;
		std::vector<int> primary_floors;
		std::vector<int> unstable_floors;

		Grammar::stabilize_Settings stabilize_settings;
		bool remove_superfluous_trusses;
		double singular;

        std::map<Components::Point*, std::vector<unsigned int> > free_dofs;
		std::vector<Components::Point*> dof_ux;
		std::vector<Components::Point*> dof_uy;
		std::vector<Components::Point*> dof_uz;
        std::map<std::pair<Components::Point*, unsigned int>, double> singular_values;

        std::map<Components::Point*, spatial_design::Geometry::Vertex*> point_vertex;
		std::map<spatial_design::Geometry::Vertex*, Components::Point*> vertex_point;
        std::map<Components::Point*, std::vector<spatial_design::Geometry::Rectangle*> > point_rectangles;
		std::map<spatial_design::Geometry::Rectangle*, std::vector<Components::Point*> > rectangle_points;

		std::vector<std::pair<Components::Point*, Components::Point*> > added_beams;
		std::vector<spatial_design::Geometry::Rectangle*> primary_rectangles;
    public:
		stabilize(SD_Analysis_Vars*, spatial_design::MS_Conformal*); // ctor
		stabilize(SD_Analysis_Vars*, spatial_design::MS_Conformal*, spatial_design::zoning::zoned_Design*); // ctor
		~stabilize(); // dtor

		// Functions called from grammar:
		void update_free_dofs(std::map<Components::Point*, std::vector<unsigned int> >);
		bool stabilize_free_dofs(unsigned int);
		void stabilize_free_dofs_zoned(unsigned int);

		// Geometrical keypoints:
		std::vector<Components::Point*> search_keypoints_truss(std::pair<Components::Point*, unsigned int>);
		std::vector<Components::Point*> search_keypoints_beam(std::pair<Components::Point*, unsigned int> point_dof);
		std::vector<Components::Point*> get_keypoints_truss(std::pair<Components::Point*, unsigned int>, char, char);
		std::vector<Components::Point*> get_keypoints_beam(std::pair<Components::Point*, unsigned int>, char);

		// Keypoint settings (deletion and ordering):
		std::vector<Components::Point*> delete_intersecting_keypoints(Components::Point*, std::vector<Components::Point*>);
		std::vector<Components::Point*> delete_external_keypoints(Components::Point*, std::vector<Components::Point*>);
		std::vector<Components::Point*> delete_inside_keypoints(Components::Point*, std::vector<Components::Point*>);
		std::vector<Components::Point*> delete_unzoned_keypoints(Components::Point*, std::vector<Components::Point*>);
		std::vector<Components::Point*> delete_unzoned_keypoints(zone, Components::Point*, std::vector<Components::Point*>);
		std::vector<Components::Point*> delete_structural_keypoints(Components::Point*, std::vector<Components::Point*>);
		std::vector<Components::Point*> delete_free_keypoints(Components::Point*, std::vector<Components::Point*>);
		std::vector<Components::Point*> delete_free_dof_keypoints(Components::Point*, unsigned int, std::vector<Components::Point*>);
		std::vector<Components::Point*> order_keypoints_singular(Components::Point*, unsigned int, std::vector<Components::Point*>);
		std::vector<Components::Point*> order_keypoints_internal(Components::Point*, std::vector<Components::Point*>);
		std::vector<Components::Point*> order_keypoints_enveloppe(Components::Point*, std::vector<Components::Point*>);
		std::vector<Components::Point*> delete_used_keypoints_beam(Components::Point*, std::vector<Components::Point*>);
		std::vector<Components::Point*> delete_unzoned_keypoints_beam(Components::Point*, std::vector<Components::Point*>);

		// Structural adjustments:
		void add_truss(std::pair<Components::Point*, Components::Point*>);
		void add_truss(spatial_design::Geometry::Rectangle*);
		void add_beams(Components::Point*, std::vector<Components::Point*>);
		void add_beam(Components::Point*, Components::Point*);
		void delete_superfluous_truss(Components::Point*, std::vector<Components::Point*>);
		void delete_superfluous_trusses();
		void delete_truss(spatial_design::Geometry::Rectangle*);

		// Some practical functions:
		void relate_points_geometry();
		bool find_rectangle(Components::Point*, Components::Point*);
		spatial_design::Geometry::Rectangle* get_rectangle(Components::Point*, Components::Point*);
		void update_singular_values();
		std::vector<Components::Point*> get_zone_points(spatial_design::zoning::zone*);
		void delete_primary_zone_dofs();
		void get_floor_grid();
		void get_floor_point();
		void check_floating_zones();

		// Output:
		void show_free_dofs();
		void show_singulars();
    }; // stabilize

    stabilize::stabilize(SD_Analysis_Vars* SD, spatial_design::MS_Conformal* CF)
    {
        m_SD = SD;
        m_CF = CF;
        m_GR = new grid(m_SD);
        grid = m_GR->get_grid();
        points_grid = m_GR->get_points_grid();
		grid_points = m_GR->get_grid_points();
		x_size = m_GR->get_grid_size('x');
		y_size = m_GR->get_grid_size('y');
		z_size = m_GR->get_grid_size('z');
        m_points = m_SD->get_points();
	    stabilize::relate_points_geometry();
		stabilize_settings = Grammar::read_stabilize_settings("Settings/stabilize_Settings.txt"); // read the stabilize settings file
		remove_superfluous_trusses = stabilize_settings.delete_superfluous_trusses;
		singular = stabilize_settings.singular;
    } // ctor

    stabilize::stabilize(SD_Analysis_Vars* SD, spatial_design::MS_Conformal* CF, spatial_design::zoning::zoned_Design* zoned_Design)
    {
        m_SD = SD;
        m_CF = CF;
		zoned = zoned_Design;
        m_GR = new grid(m_SD);
        grid = m_GR->get_grid();
        points_grid = m_GR->get_points_grid();
		grid_points = m_GR->get_grid_points();
		x_size = m_GR->get_grid_size('x');
		y_size = m_GR->get_grid_size('y');
		z_size = m_GR->get_grid_size('z');
        m_points = m_SD->get_points();
	    stabilize::relate_points_geometry();
		stabilize_settings = Grammar::read_stabilize_settings("Settings/stabilize_Settings.txt"); // read the stabilize settings file
		remove_superfluous_trusses = stabilize_settings.delete_superfluous_trusses;
		singular = stabilize_settings.singular;
		floors = zoned->get_zoned_floors();
		floor_coords = zoned->get_floor_coords();
		stabilize::get_floor_grid();
		stabilize::get_floor_point();

		for (unsigned int i = 0; i < zoned->get_zones().size(); i++)
		{
			zone temp;
			temp.zone = zoned->get_zones()[i];
			temp.points = stabilize::get_zone_points(temp.zone);
			temp.grid = new grid(temp.points);
	        temp.points_grid = temp.grid->get_points_grid();
			temp.grid_points = temp.grid->get_grid_points();
			temp.x_size = temp.grid->get_grid_size('x');
			temp.y_size = temp.grid->get_grid_size('y');
			temp.z_size = temp.grid->get_grid_size('z');
			temp.floor = temp.zone->get_floor();
			temp.floor_above = temp.zone->get_floor_above();
			if (temp.zone->get_type() <= 4)
			{
				temp.primary = true;
				if(std::find(primary_floors.begin(), primary_floors.end(), temp.floor) == primary_floors.end())
					primary_floors.push_back(temp.floor);
				if(std::find(primary_floors.begin(), primary_floors.end(), temp.floor_above) == primary_floors.end())
					primary_floors.push_back(temp.floor_above);
			}
			zones.push_back(temp);
		}
	    for (unsigned int i = 0; i < zoned->get_rectangle_count(); i++)
	    {
	        for (unsigned int j = 0; j < 4; j++) // 4 lines in rectangle
	        {
	        	zoned->get_rectangle(i)->get_line_ptr(j)->tag_zoned();
	        }
		}
		std::sort(primary_floors.begin(), primary_floors.end());
		stabilize::check_floating_zones();
    } // ctor

    stabilize::~stabilize()
    {

    }// dtor

	void stabilize::update_free_dofs(std::map<Components::Point*, std::vector<unsigned int> > points_with_free_dofs)
	{
		free_dofs = points_with_free_dofs;

		for (unsigned int i = 0; i < zones.size(); i++)
		{
			zones[i].stable_bottom = true;
			zones[i].stable_top = true;
			zones[i].stable = true;
		}

		dof_ux.clear();
		dof_uy.clear();
		dof_uz.clear();
		unstable_floors.clear();
		std::map<Components::Point*, std::vector<unsigned int> >::iterator it_1; // free_dofs
		std::map<Components::Point*, int>::iterator it_2; // point_floor
		for (it_1 = free_dofs.begin(); it_1 != free_dofs.end(); it_1++)
		{
			it_2 = point_floor.find(it_1->first);
			if (it_2 != point_floor.end())
			{
				if (std::find(unstable_floors.begin(), unstable_floors.end(), it_2->second) == unstable_floors.end())
					unstable_floors.push_back(it_2->second);
				for (unsigned int i = 0; i < zones.size(); i++)
				{
					if (zones[i].floor == it_2->second && std::find(zones[i].points.begin(), zones[i].points.end(), it_2->first) != zones[i].points.end())
					{
						zones[i].stable_bottom = false;
						zones[i].stable = false;
					}
					else if (zones[i].floor_above == it_2->second && std::find(zones[i].points.begin(), zones[i].points.end(), it_2->first) != zones[i].points.end())
					{
						zones[i].stable_top = false;
						zones[i].stable = false;
					}
				}
			}
			for (unsigned int i = 0; i < it_1->second.size(); i++)
			{
				if (it_1->second[i] == 0)
					dof_ux.push_back(it_1->first);
				else if (it_1->second[i] == 1)
					dof_uy.push_back(it_1->first);
				else if (it_1->second[i] == 2)
					dof_uz.push_back(it_1->first);

			}
		}
		std::sort(unstable_floors.begin(), unstable_floors.end());
	} // update_free_dofs()

	void stabilize::stabilize_free_dofs_zoned(unsigned int method)
	{

		std::map<std::vector<unsigned int>, Components::Point*>::iterator it_1; // grid_points
		std::map<Components::Point*, std::vector<unsigned int> >::iterator it_2; // free_dofs
		std::map<Components::Point*, std::vector<unsigned int> >::iterator it_3; // points_grid
		Components::Point* point;
		unsigned int dof;
		std::vector<unsigned int> point_grid;
		std::pair<Components::Point*, unsigned int> point_dof;
		std::vector<Components::Point*> keypoints;
		std::pair<Components::Point*, Components::Point*> dof_key;

		switch (method)
        {
        case 0: // Blind stabilization of all zones
		{
			//stabilize::check_zone_dofs();
			// check all zones for free-DOF-points without truss-keypoints and stabilize them with beams
			for (unsigned int i = 0; i < zones.size(); i++)
			{
				if (zones[i].stable == false)
				{
					for (it_2 = free_dofs.begin(); it_2 != free_dofs.end(); it_2++)
					{
						point = it_2->first;
						if (std::find(zones[i].points.begin(), zones[i].points.end(), point) != zones[i].points.end())
						{
							for (unsigned int k = 0; k < it_2->second.size(); k++)
							{
								dof = it_2->second[k];
								point_dof = std::make_pair(point, dof);
								keypoints = stabilize::search_keypoints_truss(point_dof);
								keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
								for (unsigned int j = 0; j < keypoints.size(); j++)
								{
									if (std::find(zones[i].points.begin(), zones[i].points.end(), keypoints[j]) == zones[i].points.end())
									{
										keypoints.erase(keypoints.begin() + j);
						                j--;
									}
								}
								if (keypoints.size() == 0)
			                    {
			                        keypoints = stabilize::search_keypoints_beam(point_dof);
			                        keypoints = stabilize::delete_used_keypoints_beam(point, keypoints);
			                        if (keypoints.size() > 0)
									{
			                            stabilize::add_beams(point, keypoints);
										if (remove_superfluous_trusses == true)
											stabilize::delete_superfluous_truss(point, keypoints);
									}
			                    }
							}
						}
					}
				}
			}

			for (unsigned int i = 0; i < zones.size(); i++)
			{
				// stabilize vertical planes around corners
				unsigned int z_grid = zones[i].z_size-1;
				while (z_grid > 0)
				{
		            for (unsigned int j = 0; j < 4; j++)
					{
						unsigned int l = 0;
						unsigned int m = 0;
						unsigned int n = 0;
						switch (j)
						{
							case 0: l = 0; m = 0; n = z_grid; break;
							case 1: l = 0; m = zones[i].y_size-1; n = z_grid; break;
							case 2: l = zones[i].x_size-1; m = 0; n = z_grid; break;
							case 3: l = zones[i].x_size-1; m = zones[i].y_size-1; n = z_grid; break;
						}
						point_grid = {l, m, n};
						it_1 = zones[i].grid_points.find(point_grid);
						point = it_1->second;
						dof = 0;
						point_dof = std::make_pair(point, dof);

						keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
						keypoints = stabilize::delete_structural_keypoints(point, keypoints);
						keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
						keypoints = stabilize::delete_external_keypoints(point, keypoints);
						if (keypoints.size() > 0)
						{
							dof_key = std::make_pair(point, keypoints.front());
							stabilize::add_truss(dof_key);
						}
						keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
						keypoints = stabilize::delete_structural_keypoints(point, keypoints);
						keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
						keypoints = stabilize::delete_external_keypoints(point, keypoints);
						if (keypoints.size() > 0)
						{
							dof_key = std::make_pair(point, keypoints.front());
							stabilize::add_truss(dof_key);
						}
					}
					z_grid--;
				}


				// top
				for (unsigned int l = 0; l < zones[i].x_size; l++)
	            {
					for (unsigned int m = 0; m < zones[i].y_size; m++)
		            {
						if (l == 0 || l == zones[i].x_size-1 || m == 0 || m == zones[i].y_size-1)
						{
							unsigned int n = zones[i].z_size-1;
							point_grid = {l, m, n};
							it_1 = zones[i].grid_points.find(point_grid);
							if (it_1 != zones[i].grid_points.end())
							{
								point = it_1->second;
								dof = 1;
								point_dof = std::make_pair(point, dof);
								keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
								keypoints = stabilize::delete_structural_keypoints(point, keypoints);
								keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
								keypoints = stabilize::delete_external_keypoints(point, keypoints);
								if (keypoints.size() > 0)
								{
									dof_key = std::make_pair(point, keypoints.front());
									stabilize::add_truss(dof_key);
								}
							}
						}
					}
				}
			}

			break;
		} // case 0

		case 1:
		{
			for (unsigned int p = 0; p < primary_floors.size(); p++)
			{
				if (std::find(unstable_floors.begin(), unstable_floors.end(), primary_floors[p]) != unstable_floors.end())
				{
	                for (unsigned int i = 0; i < zones.size(); i++)
	                {
	                    if (zones[i].primary == true && zones[i].floor == primary_floors[p] && zones[i].stable_bottom == false)
	                    {
	                        // bottom
	                        for (unsigned int l = 0; l < zones[i].x_size; l++)
	                        {
	                            for (unsigned int m = 0; m < zones[i].y_size; m++)
	                            {
	                                if (l == 0 || l == zones[i].x_size-1 || m == 0 || m == zones[i].y_size-1)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 1;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
	                                }
	                                if (zones[i].north_floating == true && m == zones[i].y_size-1)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                                if (zones[i].east_floating == true && l == zones[i].x_size-1)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                                if (zones[i].south_floating == true && m == 0)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                                if (zones[i].west_floating == true && l == 0)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                            }
	                        } // bottom
	                    }
	                    if (zones[i].primary == true && zones[i].floor_above == primary_floors[p] && zones[i].stable_top == false)
	                    {
	                        // top
	                        unsigned int z_grid = zones[i].z_size-1;
	                        while (z_grid > 0)
	                        {
	                        for (unsigned int l = 0; l < zones[i].x_size; l++)
	                        {
	                            for (unsigned int m = 0; m < zones[i].y_size; m++)
	                            {
	                                if (l == 0 || l == zones[i].x_size-1 || m == 0 || m == zones[i].y_size-1)
	                                {
	                                    unsigned int n = z_grid;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 1;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
	                                }
	                            }
							}
							z_grid--;
	                        } // top
	                        // vertical
	                        z_grid = zones[i].z_size-1;
	                        while (z_grid > 0)
	                        {
	                            for (unsigned int j = 0; j < 4; j++)
	                            {
	                                unsigned int l = 0;
	                                unsigned int m = 0;
	                                unsigned int n = 0;
	                                switch (j)
	                                {
	                                    case 0: l = 0; m = 0; n = z_grid; break;
	                                    case 1: l = 0; m = zones[i].y_size-1; n = z_grid; break;
	                                    case 2: l = zones[i].x_size-1; m = 0; n = z_grid; break;
	                                    case 3: l = zones[i].x_size-1; m = zones[i].y_size-1; n = z_grid; break;
	                                }
	                                point_grid = {l, m, n};
	                                it_1 = zones[i].grid_points.find(point_grid);
	                                point = it_1->second;
	                                dof = 0;
	                                point_dof = std::make_pair(point, dof);

	                                keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
                                    keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
									if (keypoints.size() > 0)
										keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
									if (keypoints.size() > 0)
										keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                if (keypoints.size() > 0)
	                                {
	                                    dof_key = std::make_pair(point, keypoints.front());
	                                    stabilize::add_truss(dof_key);
	                                }
	                                keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
                                    keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
									if (keypoints.size() > 0)
										keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
									if (keypoints.size() > 0)
										keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                if (keypoints.size() > 0)
	                                {
	                                    dof_key = std::make_pair(point, keypoints.front());
	                                    stabilize::add_truss(dof_key);
	                                }
	                            }
	                            z_grid--;
	                        } // vertical
	                    }
	                }
				}
			}
			m_SD->remesh();
			stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));

			// check design for free-DOF-points without truss-keypoints and stabilize them with beams
			for (unsigned int i = 0; i < zones.size(); i++)
			{
				if (zones[i].stable == false && zones[i].primary == true)
				{
					for (unsigned int j = 0; j < zones[i].points.size(); j++)
					{
						it_2 = free_dofs.find(zones[i].points[j]);
						if (it_2 != free_dofs.end())
						{
							point = it_2->first;
							for (unsigned int k = 0; k < it_2->second.size(); k++)
							{
								dof = it_2->second[k];
								point_dof = std::make_pair(point, dof);
								keypoints = stabilize::search_keypoints_truss(point_dof);
								keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
								//keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);

								if (keypoints.size() == 0)
			                    {
			                        keypoints = stabilize::search_keypoints_beam(point_dof);
			                        keypoints = stabilize::delete_used_keypoints_beam(point, keypoints);
									keypoints = stabilize::delete_unzoned_keypoints_beam(point, keypoints);
			                        if (keypoints.size() > 0)
									{
			                            stabilize::add_beams(point, keypoints);
										if (remove_superfluous_trusses == true)
											stabilize::delete_superfluous_truss(point, keypoints);
									}
			                    }
							}
						}
					}

				}
			}

			m_SD->remesh();
			stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));
			for (unsigned int i = 0; i < zones.size(); i++)
			{
				if (zones[i].stable == false && zones[i].primary == false)
				{
					for (unsigned int j = 0; j < zones[i].points.size(); j++)
					{
						it_2 = free_dofs.find(zones[i].points[j]);
						if (it_2 != free_dofs.end())
						{
							point = it_2->first;
							for (unsigned int k = 0; k < it_2->second.size(); k++)
							{
								dof = it_2->second[k];
								point_dof = std::make_pair(point, dof);
								keypoints = stabilize::search_keypoints_truss(point_dof);
								keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
								//keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);

								if (keypoints.size() == 0)
			                    {
			                        keypoints = stabilize::search_keypoints_beam(point_dof);
			                        keypoints = stabilize::delete_used_keypoints_beam(point, keypoints);
									keypoints = stabilize::delete_unzoned_keypoints_beam(point, keypoints);
			                        if (keypoints.size() > 0)
									{
			                            stabilize::add_beams(point, keypoints);
										if (remove_superfluous_trusses == true)
											stabilize::delete_superfluous_truss(point, keypoints);
									}
			                    }
							}
						}
					}

				}
			}

			break;
		} // case 1

		case 2:
		{
			for (unsigned int p = 0; p < primary_floors.size(); p++)
			{
				if (std::find(unstable_floors.begin(), unstable_floors.end(), primary_floors[p]) != unstable_floors.end())
				{
	                for (unsigned int i = 0; i < zones.size(); i++)
	                {
	                    if (zones[i].primary == true && zones[i].floor == primary_floors[p] && zones[i].stable_bottom == false)
	                    {
	                        // bottom
	                        for (unsigned int l = 0; l < zones[i].x_size; l++)
	                        {
	                            for (unsigned int m = 0; m < zones[i].y_size; m++)
	                            {
	                                if (l == 0 || l == zones[i].x_size-1 || m == 0 || m == zones[i].y_size-1)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 1;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
	                                }
	                                if (zones[i].north_floating == true && m == zones[i].y_size-1)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                                if (zones[i].east_floating == true && l == zones[i].x_size-1)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                                if (zones[i].south_floating == true && m == 0)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                                if (zones[i].west_floating == true && l == 0)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                            }
	                        } // bottom
	                    }
	                    if (zones[i].primary == true && zones[i].floor_above == primary_floors[p] && zones[i].stable_top == false)
	                    {
	                        // top
	                        unsigned int z_grid = zones[i].z_size-1;
	                        while (z_grid > 0)
	                        {
	                        for (unsigned int l = 0; l < zones[i].x_size; l++)
	                        {
	                            for (unsigned int m = 0; m < zones[i].y_size; m++)
	                            {
	                                if (l == 0 || l == zones[i].x_size-1 || m == 0 || m == zones[i].y_size-1)
	                                {
	                                    unsigned int n = z_grid;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 1;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
	                                }
	                            }
							}
							z_grid--;
	                        } // top
	                        // vertical
	                        z_grid = zones[i].z_size-1;
	                        while (z_grid > 0)
	                        {
	                            for (unsigned int j = 0; j < 4; j++)
	                            {
	                                unsigned int l = 0;
	                                unsigned int m = 0;
	                                unsigned int n = 0;
	                                switch (j)
	                                {
	                                    case 0: l = 0; m = 0; n = z_grid; break;
	                                    case 1: l = 0; m = zones[i].y_size-1; n = z_grid; break;
	                                    case 2: l = zones[i].x_size-1; m = 0; n = z_grid; break;
	                                    case 3: l = zones[i].x_size-1; m = zones[i].y_size-1; n = z_grid; break;
	                                }
	                                point_grid = {l, m, n};
	                                it_1 = zones[i].grid_points.find(point_grid);
	                                point = it_1->second;
	                                dof = 0;
	                                point_dof = std::make_pair(point, dof);

	                                keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
                                    keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
									if (keypoints.size() > 0)
										keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
									if (keypoints.size() > 0)
										keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                if (keypoints.size() > 0)
	                                {
	                                    dof_key = std::make_pair(point, keypoints.front());
	                                    stabilize::add_truss(dof_key);
	                                }
	                                keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
                                    keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
									if (keypoints.size() > 0)
										keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
									if (keypoints.size() > 0)
										keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                if (keypoints.size() > 0)
	                                {
	                                    dof_key = std::make_pair(point, keypoints.front());
	                                    stabilize::add_truss(dof_key);
	                                }
	                            }
	                            z_grid--;
	                        } // vertical
	                    }
	                }
				}
			}
			m_SD->remesh();
			stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));

			for (unsigned int p = 0; p < floors.size(); p++)
			{
                for (unsigned int i = 0; i < zones.size(); i++)
                {
                    if (zones[i].primary == false && zones[i].floor == floors[p] && zones[i].stable_bottom == false)
                    {
                        // bottom
                        for (unsigned int l = 0; l < zones[i].x_size; l++)
                        {
                            for (unsigned int m = 0; m < zones[i].y_size; m++)
                            {
                                if (l == 0 || l == zones[i].x_size-1 || m == 0 || m == zones[i].y_size-1)
                                {
                                    unsigned int n = 0;
                                    point_grid = {l, m, n};
                                    it_1 = zones[i].grid_points.find(point_grid);
                                    if (it_1 != zones[i].grid_points.end())
                                    {
                                        point = it_1->second;
                                        dof = 1;
                                        point_dof = std::make_pair(point, dof);
                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
										if (keypoints.size() > 0)
											keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
										if (keypoints.size() > 0)
											keypoints = stabilize::delete_structural_keypoints(point, keypoints);
                                        if (keypoints.size() > 0)
                                        {
                                            dof_key = std::make_pair(point, keypoints.front());
                                            stabilize::add_truss(dof_key);
                                        }
                                    }
	                                if (zones[i].north_floating == true && m == zones[i].y_size-1)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                                if (zones[i].east_floating == true && l == zones[i].x_size-1)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                                if (zones[i].south_floating == true && m == 0)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
	                                if (zones[i].west_floating == true && l == 0)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 2;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
											if (keypoints.size() > 0)
												keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
									}
                                }
                            }
                        } // bottom
						//m_SD->remesh();
						//stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));
                    }
                    if (zones[i].primary == false && zones[i].floor_above == floors[p] && zones[i].stable_top == false)
                    {
                        // top
                        unsigned int z_grid = zones[i].z_size-1;
                        while (z_grid > 0)
                        {
                        for (unsigned int l = 0; l < zones[i].x_size; l++)
                        {
                            for (unsigned int m = 0; m < zones[i].y_size; m++)
                            {
                                if (l == 0 || l == zones[i].x_size-1 || m == 0 || m == zones[i].y_size-1)
                                {
                                    unsigned int n = z_grid;
                                    point_grid = {l, m, n};
                                    it_1 = zones[i].grid_points.find(point_grid);
                                    if (it_1 != zones[i].grid_points.end())
                                    {
                                        point = it_1->second;
                                        dof = 1;
                                        point_dof = std::make_pair(point, dof);
                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
										if (keypoints.size() > 0)
											keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
										if (keypoints.size() > 0)
											keypoints = stabilize::delete_structural_keypoints(point, keypoints);
                                        if (keypoints.size() > 0)
                                        {
                                            dof_key = std::make_pair(point, keypoints.front());
                                            stabilize::add_truss(dof_key);
                                        }
                                    }
                                }
                            }
						}
						z_grid--;
                        } // top
						//m_SD->remesh();
						//stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));
					}
                } // zones
			} // floors
			m_SD->remesh();
			stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));

			for (unsigned int p = 0; p < floors.size(); p++)
			{
                for (unsigned int i = 0; i < zones.size(); i++)
                {
		            if (zones[i].primary == false && zones[i].floor_above == floors[p] && zones[i].stable_top == false)
		            {
						// vertical
		                unsigned int z_grid = zones[i].z_size-1;
		                while (z_grid > 0)
		                {
		                    for (unsigned int j = 0; j < 4; j++)
		                    {
		                        unsigned int l = 0;
		                        unsigned int m = 0;
		                        unsigned int n = 0;
		                        switch (j)
		                        {
		                            case 0: l = 0; m = 0; n = z_grid; break;
		                            case 1: l = 0; m = zones[i].y_size-1; n = z_grid; break;
		                            case 2: l = zones[i].x_size-1; m = 0; n = z_grid; break;
		                            case 3: l = zones[i].x_size-1; m = zones[i].y_size-1; n = z_grid; break;
		                        }
		                        point_grid = {l, m, n};
		                        it_1 = zones[i].grid_points.find(point_grid);
		                        point = it_1->second;
		                        dof = 0;
		                        point_dof = std::make_pair(point, dof);

		                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
		                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
								if (keypoints.size() > 0)
									keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
								if (keypoints.size() > 0)
									keypoints = stabilize::delete_structural_keypoints(point, keypoints);
		                        if (keypoints.size() > 0)
		                        {
		                            dof_key = std::make_pair(point, keypoints.front());
		                            stabilize::add_truss(dof_key);
		                        }
		                        keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
		                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
								if (keypoints.size() > 0)
									keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
								if (keypoints.size() > 0)
									keypoints = stabilize::delete_structural_keypoints(point, keypoints);
		                        if (keypoints.size() > 0)
		                        {
		                            dof_key = std::make_pair(point, keypoints.front());
		                            stabilize::add_truss(dof_key);
		                        }
		                    }
		                    z_grid--;
		                } // vertical
		            }
                } // zones
				m_SD->remesh();
				stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));
            } // floors

			// check design for free-DOF-points without truss-keypoints and stabilize them with beams
			for (unsigned int i = 0; i < zones.size(); i++)
			{
				if (zones[i].stable == false && zones[i].primary == true)
				{
					for (unsigned int j = 0; j < zones[i].points.size(); j++)
					{
						it_2 = free_dofs.find(zones[i].points[j]);
						if (it_2 != free_dofs.end())
						{
							point = it_2->first;
							for (unsigned int k = 0; k < it_2->second.size(); k++)
							{
								dof = it_2->second[k];
								point_dof = std::make_pair(point, dof);
								keypoints = stabilize::search_keypoints_truss(point_dof);
								keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
								//keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);

								if (keypoints.size() == 0)
			                    {
			                        keypoints = stabilize::search_keypoints_beam(point_dof);
			                        keypoints = stabilize::delete_used_keypoints_beam(point, keypoints);
									keypoints = stabilize::delete_unzoned_keypoints_beam(point, keypoints);
			                        if (keypoints.size() > 0)
									{
			                            stabilize::add_beams(point, keypoints);
										if (remove_superfluous_trusses == true)
											stabilize::delete_superfluous_truss(point, keypoints);
									}
			                    }
							}
						}
					}

				}
			}

			m_SD->remesh();
			stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));
			for (unsigned int i = 0; i < zones.size(); i++)
			{
				if (zones[i].stable == false && zones[i].primary == false)
				{
					for (unsigned int j = 0; j < zones[i].points.size(); j++)
					{
						it_2 = free_dofs.find(zones[i].points[j]);
						if (it_2 != free_dofs.end())
						{
							point = it_2->first;
							for (unsigned int k = 0; k < it_2->second.size(); k++)
							{
								dof = it_2->second[k];
								point_dof = std::make_pair(point, dof);
								keypoints = stabilize::search_keypoints_truss(point_dof);
								keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
								//keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);

								if (keypoints.size() == 0)
			                    {
			                        keypoints = stabilize::search_keypoints_beam(point_dof);
			                        keypoints = stabilize::delete_used_keypoints_beam(point, keypoints);
									keypoints = stabilize::delete_unzoned_keypoints_beam(point, keypoints);
			                        if (keypoints.size() > 0)
									{
			                            stabilize::add_beams(point, keypoints);
										if (remove_superfluous_trusses == true)
											stabilize::delete_superfluous_truss(point, keypoints);
									}
			                    }
							}
						}
					}

				}
			}

			/*
			for (it_2 = free_dofs.begin(); it_2 != free_dofs.end(); it_2++)
			{
				point = it_2->first;
				for (unsigned int k = 0; k < it_2->second.size(); k++)
				{
					dof = it_2->second[k];
					point_dof = std::make_pair(point, dof);
					keypoints = stabilize::search_keypoints_truss(point_dof);
					keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);

					if (keypoints.size() == 0)
                    {
                        keypoints = stabilize::search_keypoints_beam(point_dof);
                        keypoints = stabilize::delete_used_keypoints_beam(point, keypoints);
						keypoints = stabilize::delete_unzoned_keypoints_beam(point, keypoints);
                        if (keypoints.size() > 0)
						{
                            stabilize::add_beams(point, keypoints);
							if (remove_superfluous_trusses == true)
								stabilize::delete_superfluous_truss(point, keypoints);
						}
                    }
				}
			}
			*/
			break;
		} // case 2

		case 3:
		{
			for (unsigned int p = 0; p < primary_floors.size(); p++)
			{
				if (std::find(unstable_floors.begin(), unstable_floors.end(), primary_floors[p]) != unstable_floors.end())
				{
	                for (unsigned int i = 0; i < zones.size(); i++)
	                {
	                    if (zones[i].primary == false && zones[i].floor_above == primary_floors[p] && zones[i].stable_top == false)
	                    {
	                        // top
	                        for (unsigned int l = 0; l < zones[i].x_size; l++)
	                        {
	                            for (unsigned int m = 0; m < zones[i].y_size; m++)
	                            {
	                                if (l == 0 || l == zones[i].x_size-1 || m == 0 || m == zones[i].y_size-1)
	                                {
	                                    unsigned int n = zones[i].z_size-1;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 1;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
	                                        keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
	                                        keypoints = stabilize::delete_external_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
	                                }
	                            }
	                        } // top
							m_SD->remesh();
							stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));
	                    }
					}
				}
			}
			for (unsigned int p = 0; p < primary_floors.size(); p++)
			{
				if (std::find(unstable_floors.begin(), unstable_floors.end(), primary_floors[p]) != unstable_floors.end())
				{
	                for (unsigned int i = 0; i < zones.size(); i++)
	                {
	                    if (zones[i].primary == false && zones[i].floor_above == primary_floors[p] && zones[i].stable_top == false)
	                    {
	                        // vertical
	                        unsigned int z_grid = zones[i].z_size-1;
	                        while (z_grid > 0)
	                        {
	                            for (unsigned int j = 0; j < 4; j++)
	                            {
	                                unsigned int l = 0;
	                                unsigned int m = 0;
	                                unsigned int n = 0;
	                                switch (j)
	                                {
	                                    case 0: l = 0; m = 0; n = z_grid; break;
	                                    case 1: l = 0; m = zones[i].y_size-1; n = z_grid; break;
	                                    case 2: l = zones[i].x_size-1; m = 0; n = z_grid; break;
	                                    case 3: l = zones[i].x_size-1; m = zones[i].y_size-1; n = z_grid; break;
	                                }
	                                point_grid = {l, m, n};
	                                it_1 = zones[i].grid_points.find(point_grid);
	                                point = it_1->second;
	                                dof = 0;
	                                point_dof = std::make_pair(point, dof);

	                                keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
	                                keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
	                                keypoints = stabilize::delete_external_keypoints(point, keypoints);
	                                if (keypoints.size() > 0)
	                                {
	                                    dof_key = std::make_pair(point, keypoints.front());
	                                    stabilize::add_truss(dof_key);
	                                }
	                                keypoints = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
	                                keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
	                                keypoints = stabilize::delete_external_keypoints(point, keypoints);
	                                if (keypoints.size() > 0)
	                                {
	                                    dof_key = std::make_pair(point, keypoints.front());
	                                    stabilize::add_truss(dof_key);
	                                }
	                            }
	                            z_grid--;
	                        } // vertical
							m_SD->remesh();
							stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));
	                    }
					}
				}
			}
			for (unsigned int p = 0; p < primary_floors.size(); p++)
			{
				if (std::find(unstable_floors.begin(), unstable_floors.end(), primary_floors[p]) != unstable_floors.end())
				{
	                for (unsigned int i = 0; i < zones.size(); i++)
	                {
	                    if (zones[i].primary == false && zones[i].floor == primary_floors[p] && zones[i].stable_bottom == false)
	                    {
	                        // bottom
	                        for (unsigned int l = 0; l < zones[i].x_size; l++)
	                        {
	                            for (unsigned int m = 0; m < zones[i].y_size; m++)
	                            {
	                                if (l == 0 || l == zones[i].x_size-1 || m == 0 || m == zones[i].y_size-1)
	                                {
	                                    unsigned int n = 0;
	                                    point_grid = {l, m, n};
	                                    it_1 = zones[i].grid_points.find(point_grid);
	                                    if (it_1 != zones[i].grid_points.end())
	                                    {
	                                        point = it_1->second;
	                                        dof = 1;
	                                        point_dof = std::make_pair(point, dof);
	                                        keypoints = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
	                                        keypoints = stabilize::delete_structural_keypoints(point, keypoints);
	                                        keypoints = stabilize::delete_unzoned_keypoints(zones[i], point, keypoints);
	                                        keypoints = stabilize::delete_external_keypoints(point, keypoints);
	                                        if (keypoints.size() > 0)
	                                        {
	                                            dof_key = std::make_pair(point, keypoints.front());
	                                            stabilize::add_truss(dof_key);
	                                        }
	                                    }
	                                }
	                            }
	                        } // bottom
							m_SD->remesh();
							stabilize::update_free_dofs(m_SD->get_zoned_points_with_free_dofs(singular));
	                    }
					}
				}
			}
			break;
		}

		default:
        	std::cerr << "No method for stabilization sequence chosen, exiting now..." << std::endl;
        	exit(1);
		} // switch
	}

	bool stabilize::stabilize_free_dofs(unsigned int method)
	{
		bool stabilization_possible = true;

		std::vector<vector<vector<coord*>>>::iterator it_x; // grid-location x
        std::vector<vector<coord*>>::iterator it_y; // grid-location y
        std::vector<coord*>::iterator it_z; // grid-location z
		std::map<std::vector<unsigned int>, Components::Point*>::iterator it_1; // grid_points
		std::map<Components::Point*, std::vector<unsigned int> >::iterator it_2; // free_dofs
		std::map<Components::Point*, std::vector<unsigned int> >::iterator it_3; // points_grid
		Components::Point* point;
		unsigned int dof;
		std::vector<unsigned int> point_grid;
		std::pair<Components::Point*, unsigned int> point_dof;
		std::vector<Components::Point*> keypoints;
		std::pair<Components::Point*, Components::Point*> dof_key;
		bool found = false;
		switch (method)
        {
        case 0:
		/*
		Sequence:
		grid: x,y,z -> Dof: uX,uY,uZ

		Structure:
		Rod addition, Beam addition/substitution
		*/
		{
			for (it_x = grid.begin(); it_x != grid.end(); it_x++)
            {
                for (it_y = it_x->begin(); it_y != it_x->end(); it_y++)
                {
					for(it_z = it_y->begin(); it_z != it_y->end(); it_z++)
					{
						point_grid.clear();
						point_grid.push_back(std::distance(grid.begin(), it_x));
	                    point_grid.push_back(std::distance(it_x->begin(), it_y));
	                    point_grid.push_back(std::distance(it_y->begin(), it_z));
						it_1 = grid_points.find(point_grid);
						it_2 = free_dofs.find(it_1->second);
						if (it_2 != free_dofs.end())
						{
							point = it_1->second;

							for (unsigned int i = 0; i < it_2->second.size(); i++)
							{
								dof = it_2->second[i];
								point_dof = std::make_pair(point, dof);
								keypoints = stabilize::search_keypoints_truss(point_dof);
								if (keypoints.size() > 0)
								{
									// delete and order keypoints:
									//keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
									keypoints = stabilize::delete_intersecting_keypoints(point, keypoints);
									keypoints = stabilize::delete_external_keypoints(point, keypoints);
									keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);
									keypoints = stabilize::delete_structural_keypoints(point, keypoints);

									if (keypoints.size() > 0)
									{
									    found = true;
									    dof_key = std::make_pair(point, keypoints.front());
									}
								}
								if (found == true)
									break;
							}
						}
						if (found == true)
							break;
					}
					if (found == true)
						break;
				}
				if (found == true)
					break;
			}
			if (found == true)
			{
				stabilize::add_truss(dof_key);
			}

			else // Structure cannot be stabilized by truss addition; delete trusses/add beams
            {
                for (it_x = grid.begin(); it_x != grid.end(); it_x++)
                {
                    for (it_y = it_x->begin(); it_y != it_x->end(); it_y++)
                    {
                        for(it_z = it_y->begin(); it_z != it_y->end(); it_z++)
                        {
                            point_grid.clear();
                            point_grid.push_back(std::distance(grid.begin(), it_x));
                            point_grid.push_back(std::distance(it_x->begin(), it_y));
                            point_grid.push_back(std::distance(it_y->begin(), it_z));
                            it_1 = grid_points.find(point_grid);
                            it_2 = free_dofs.find(it_1->second);
                            if (it_2 != free_dofs.end())
                            {
                                point = it_1->second;

                                for (unsigned int i = 0; i < it_2->second.size(); i++)
                                {
                                    dof = it_2->second[i];
                                    point_dof = std::make_pair(point, dof);
                                    keypoints = stabilize::search_keypoints_beam(point_dof);
                                   if (keypoints.size() > 0)
                                    {
                                        // delete and order keypoints:
                                        //keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);
										keypoints = stabilize::delete_used_keypoints_beam(point, keypoints);
										//keypoints = stabilize::delete_unzoned_keypoints_beam(point, keypoints);

                                        if (keypoints.size() > 0)
                                        {
                                            found = true;
                                            break;
                                        }
                                    }
                                    if (found == true)
                                        break;
                                }
                            }
                            if (found == true)
                                break;
                        }
                        if (found == true)
                            break;
                    }
                    if (found == true)
                        break;
                }
                if (found == true)
                {
                    stabilize::add_beams(point, keypoints);
                }
				else
				{
		        	std::cout << "Structure can not be stabilized..." << std::endl;
		        	stabilization_possible = false;
				}
				if (remove_superfluous_trusses == true)
					stabilize::delete_superfluous_trusses();
            }
			break;
		} // case 0

        case 1:
		/*
		Sequence:
		grid: zmin,x,y -> Dof: uX,uY,uZ

		Structure:
		Truss addition, Beam addition/substitution
		*/
		{
			//stabilize::delete_primary_zone_dofs();
			for (unsigned int n = 0; n < z_size; n++)
            {
                for (unsigned int l = 0; l < x_size; l++)
                {
					for(unsigned int m = 0; m < y_size; m++)
					{
						//unsigned int n = N;
						point_grid = {l, m, n};
						it_1 = grid_points.find(point_grid);
						it_2 = free_dofs.find(it_1->second);
						if (it_2 != free_dofs.end())
						{
							point = it_1->second;

							for (unsigned int i = 0; i < it_2->second.size(); i++)
							{
								dof = it_2->second[i];
								point_dof = std::make_pair(point, dof);
								keypoints = stabilize::search_keypoints_truss(point_dof);
								if (keypoints.size() > 0)
								{
									// delete and order keypoints:
									keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
									//keypoints = stabilize::delete_intersecting_keypoints(point, keypoints);
									//keypoints = stabilize::delete_external_keypoints(point, keypoints);
									keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);
									keypoints = stabilize::delete_structural_keypoints(point, keypoints);

									if (keypoints.size() > 0)
									{
									    found = true;
									    dof_key = std::make_pair(point, keypoints.front());
									}
								}
								if (found == true)
									break;
							}
						}
						if (found == true)
							break;
					}
					if (found == true)
						break;
				}
				if (found == true)
					break;
			}
			if (found == true)
			{
				stabilize::add_truss(dof_key);
			}

			else // Structure cannot be stabilized by truss addition; delete trusses/add beams
            {
				for (int N = z_size-1; N >= 0; N--)
	            {
	                for (unsigned int l = 0; l < x_size; l++)
	                {
						for(unsigned int m = 0; m < y_size; m++)
						{
							unsigned int n = N;
							point_grid = {m, l, n};
                            it_1 = grid_points.find(point_grid);
                            it_2 = free_dofs.find(it_1->second);
                            if (it_2 != free_dofs.end())
                            {
                                point = it_1->second;

                                for (unsigned int i = 0; i < it_2->second.size(); i++)
                                {
                                    dof = it_2->second[i];
                                    point_dof = std::make_pair(point, dof);
                                    keypoints = stabilize::search_keypoints_beam(point_dof);
                                   if (keypoints.size() > 0)
                                    {
                                        // delete and order keypoints:
                                        //keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);
										keypoints = stabilize::delete_used_keypoints_beam(point, keypoints);
										keypoints = stabilize::delete_unzoned_keypoints_beam(point, keypoints);

                                        if (keypoints.size() > 0)
                                        {
                                            found = true;
                                            break;
                                        }
                                    }
                                    if (found == true)
                                        break;
                                }
                            }
                            if (found == true)
                                break;
                        }
                        if (found == true)
                            break;
                    }
                    if (found == true)
                        break;
                }
                if (found == true)
                {
                    stabilize::add_beams(point, keypoints);
                }
				else
				{
		        	stabilization_possible = false;
					std::cout << "zones can not be stabilized (end of zone point iteration)" << std::endl;
				}
				if (remove_superfluous_trusses == true)
					stabilize::delete_superfluous_trusses();
            }
			break;
		} // case 1

        case 2:
		/*
		Sequence:
		grid: zmax,x,y -> Dof: uX,uY,uZ

		Structure:
		Truss addition, Beam addition/substitution
		*/
		{
			//stabilize::delete_primary_zone_dofs();
			for (int N = z_size-1; N >= 0; N--)
            {
                for (unsigned int l = 0; l < x_size; l++)
                {
					for(unsigned int m = 0; m < y_size; m++)
					{
						unsigned int n = N;
						point_grid = {l, m, n};
						it_1 = grid_points.find(point_grid);
						it_2 = free_dofs.find(it_1->second);
						if (it_2 != free_dofs.end())
						{
							point = it_1->second;

							for (unsigned int i = 0; i < it_2->second.size(); i++)
							{
								dof = it_2->second[i];
								point_dof = std::make_pair(point, dof);
								keypoints = stabilize::search_keypoints_truss(point_dof);
								if (keypoints.size() > 0)
								{
									// delete and order keypoints:
									keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
									//keypoints = stabilize::delete_intersecting_keypoints(point, keypoints);
									//keypoints = stabilize::delete_external_keypoints(point, keypoints);
									keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);
									keypoints = stabilize::delete_structural_keypoints(point, keypoints);

									if (keypoints.size() > 0)
									{
									    found = true;
									    dof_key = std::make_pair(point, keypoints.front());
									}
								}
								if (found == true)
									break;
							}
						}
						if (found == true)
							break;
					}
					if (found == true)
						break;
				}
				if (found == true)
					break;
			}
			if (found == true)
			{
				stabilize::add_truss(dof_key);
			}

			else // Structure cannot be stabilized by truss addition; delete trusses/add beams
            {
				for (int N = z_size-1; N >= 0; N--)
	            {
	                for (unsigned int l = 0; l < x_size; l++)
	                {
						for(unsigned int m = 0; m < y_size; m++)
						{
							unsigned int n = N;
							point_grid = {m, l, n};
                            it_1 = grid_points.find(point_grid);
                            it_2 = free_dofs.find(it_1->second);
                            if (it_2 != free_dofs.end())
                            {
                                point = it_1->second;

                                for (unsigned int i = 0; i < it_2->second.size(); i++)
                                {
                                    dof = it_2->second[i];
                                    point_dof = std::make_pair(point, dof);
                                    keypoints = stabilize::search_keypoints_beam(point_dof);
                                   if (keypoints.size() > 0)
                                    {
                                        // delete and order keypoints:
                                        //keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);
										keypoints = stabilize::delete_used_keypoints_beam(point, keypoints);
										keypoints = stabilize::delete_unzoned_keypoints_beam(point, keypoints);

                                        if (keypoints.size() > 0)
                                        {
                                            found = true;
                                            break;
                                        }
                                    }
                                    if (found == true)
                                        break;
                                }
                            }
                            if (found == true)
                                break;
                        }
                        if (found == true)
                            break;
                    }
                    if (found == true)
                        break;
                }
                if (found == true)
                {
                    stabilize::add_beams(point, keypoints);
                }
				else
				{
		        	stabilization_possible = false;
					std::cout << "zones can not be stabilized (end of zone point iteration)" << std::endl;
				}
				if (remove_superfluous_trusses == true)
					stabilize::delete_superfluous_trusses();
            }
			break;
		} // case 2

        case 3:
		/*
		Sequence:
		grid: x,y,z -> Dof: uX,uY,uZ

		Structure:
		Rod addition, Beam addition/substitution
		*/
		{
			for (it_x = grid.begin(); it_x != grid.end(); it_x++)
            {
                for (it_y = it_x->begin(); it_y != it_x->end(); it_y++)
                {
					for(it_z = it_y->begin(); it_z != it_y->end(); it_z++)
					{
						point_grid.clear();
						point_grid.push_back(std::distance(grid.begin(), it_x));
	                    point_grid.push_back(std::distance(it_x->begin(), it_y));
	                    point_grid.push_back(std::distance(it_y->begin(), it_z));
						it_1 = grid_points.find(point_grid);
						it_2 = free_dofs.find(it_1->second);
						if (it_2 != free_dofs.end())
						{
							point = it_1->second;

							for (unsigned int i = 0; i < it_2->second.size(); i++)
							{
								dof = it_2->second[i];
								point_dof = std::make_pair(point, dof);
								keypoints = stabilize::search_keypoints_truss(point_dof);
								if (keypoints.size() > 0)
								{
									// delete and order keypoints:
									keypoints = stabilize::delete_unzoned_keypoints(point, keypoints);
									//keypoints = stabilize::delete_intersecting_keypoints(point, keypoints);
									keypoints = stabilize::delete_external_keypoints(point, keypoints);
									keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);
									keypoints = stabilize::delete_structural_keypoints(point, keypoints);

									if (keypoints.size() > 0)
									{
									    found = true;
									    dof_key = std::make_pair(point, keypoints.front());
									}
								}
								if (found == true)
									break;
							}
						}
						if (found == true)
							break;
					}
					if (found == true)
						break;
				}
				if (found == true)
					break;
			}
			if (found == true)
			{
				stabilize::add_truss(dof_key);
			}

			else // Structure cannot be stabilized by truss addition; delete trusses/add beams
            {
                for (it_x = grid.begin(); it_x != grid.end(); it_x++)
                {
                    for (it_y = it_x->begin(); it_y != it_x->end(); it_y++)
                    {
                        for(it_z = it_y->begin(); it_z != it_y->end(); it_z++)
                        {
                            point_grid.clear();
                            point_grid.push_back(std::distance(grid.begin(), it_x));
                            point_grid.push_back(std::distance(it_x->begin(), it_y));
                            point_grid.push_back(std::distance(it_y->begin(), it_z));
                            it_1 = grid_points.find(point_grid);
                            it_2 = free_dofs.find(it_1->second);
                            if (it_2 != free_dofs.end())
                            {
                                point = it_1->second;

                                for (unsigned int i = 0; i < it_2->second.size(); i++)
                                {
                                    dof = it_2->second[i];
                                    point_dof = std::make_pair(point, dof);
                                    keypoints = stabilize::search_keypoints_beam(point_dof);
                                   if (keypoints.size() > 0)
                                    {
                                        // delete and order keypoints:
                                        //keypoints = stabilize::delete_free_dof_keypoints(point, dof, keypoints);
										keypoints = stabilize::delete_used_keypoints_beam(point, keypoints);
										//keypoints = stabilize::delete_unzoned_keypoints_beam(point, keypoints);

                                        if (keypoints.size() > 0)
                                        {
                                            found = true;
                                            break;
                                        }
                                    }
                                    if (found == true)
                                        break;
                                }
                            }
                            if (found == true)
                                break;
                        }
                        if (found == true)
                            break;
                    }
                    if (found == true)
                        break;
                }
                if (found == true)
                {
                    stabilize::add_beams(point, keypoints);
                }
				else
				{
		        	std::cout << "Structure can not be stabilized..." << std::endl;
		        	stabilization_possible = false;
				}
				if (remove_superfluous_trusses == true)
					stabilize::delete_superfluous_trusses();
            }
			break;
		} // case 3

		default:
        	std::cerr << "No method for stabilization sequence chosen, exiting now..." << std::endl;
        	exit(1);
        } // switch (method)
		return stabilization_possible;
	} // stabilize_free_dofs()

    std::vector<Components::Point*> stabilize::delete_intersecting_keypoints(Components::Point* point, std::vector<Components::Point*> keypoints)
    {
        std::vector<Components::Point*> temp_keypoints;
		for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			if (stabilize::find_rectangle(point, keypoint) == true && stabilize::get_rectangle(point, keypoint)->get_surface_count() != 0)
			{
                temp_keypoints.push_back(keypoint);
			}
		}
		return temp_keypoints;
    } // delete_intersecting_keypoints()

	std::vector<Components::Point*> stabilize::delete_external_keypoints(Components::Point* point, std::vector<Components::Point*> keypoints)
	{
        std::vector<Components::Point*> temp_keypoints;
		for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			if (stabilize::find_rectangle(point, keypoint) == true)
			{
                temp_keypoints.push_back(keypoint);
			}
		}
		return temp_keypoints;
	} // delete_external_keypoints()

	std::vector<Components::Point*> stabilize::delete_inside_keypoints(Components::Point* point, std::vector<Components::Point*> keypoints)
	{
        std::vector<Components::Point*> temp_keypoints;
		for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			if (stabilize::find_rectangle(point, keypoint) == true && stabilize::get_rectangle(point, keypoint)->get_surface_count() == 1)
			{
                temp_keypoints.push_back(keypoint);
			}
			else if (stabilize::find_rectangle(point, keypoint) == false)
			{
                temp_keypoints.push_back(keypoint);
			}
		}
		return temp_keypoints;
	} // delete_inside_keypoints()

	std::vector<Components::Point*> stabilize::delete_unzoned_keypoints(Components::Point* point, std::vector<Components::Point*> keypoints)
    {
        std::vector<Components::Point*> temp_keypoints;
		for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			if (stabilize::find_rectangle(point, keypoint) == true && stabilize::get_rectangle(point, keypoint)->get_zoned() == true)
			{
                temp_keypoints.push_back(keypoint);
			}
		}
		return temp_keypoints;
    } // delete_unzoned_keypoints()

	std::vector<Components::Point*> stabilize::delete_unzoned_keypoints(zone temp_zone, Components::Point* point, std::vector<Components::Point*> keypoints)
    {
        std::vector<Components::Point*> temp_keypoints;
		for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			if (std::find(temp_zone.points.begin(), temp_zone.points.end(), keypoint) != temp_zone.points.end())
			{
                temp_keypoints.push_back(keypoint);
			}
		}
		return temp_keypoints;
    } // delete_unzoned_keypoints(zone)

	std::vector<Components::Point*> stabilize::delete_structural_keypoints(Components::Point* point, std::vector<Components::Point*> keypoints)
    {
        std::vector<Components::Point*> temp_keypoints;
		for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			if (stabilize::find_rectangle(point, keypoint) == true && stabilize::get_rectangle(point, keypoint)->is_structural() == false)
			{
                temp_keypoints.push_back(keypoint);
			}
			else if (stabilize::find_rectangle(point, keypoint) == false)
			{
                temp_keypoints.push_back(keypoint);
			}
		}
		return temp_keypoints;
    } // delete_structural_keypoints()

    std::vector<Components::Point*> stabilize::delete_free_keypoints(Components::Point* point, std::vector<Components::Point*> keypoints)
    {
		std::vector<Components::Point*> temp_keypoints;
		std::map<Components::Point*, std::vector<unsigned int> >::iterator it_1; // free_dofs
        for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			it_1 = free_dofs.find(keypoint);
			if (it_1 == free_dofs.end())
			{
                temp_keypoints.push_back(keypoint);
			}
		}
		return temp_keypoints;
    } // delete_free_keypoints()

	std::vector<Components::Point*> stabilize::delete_free_dof_keypoints(Components::Point* point, unsigned int dof, std::vector<Components::Point*> keypoints)
    {
		std::vector<Components::Point*> temp_keypoints;
		std::map<Components::Point*, std::vector<unsigned int> >::iterator it_1; // free_dofs
        for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			it_1 = free_dofs.find(keypoint);
			if (it_1 != free_dofs.end())
			{
                std::vector<unsigned int> dofs = it_1->second;
				if (std::find(dofs.begin(), dofs.end(), dof) == dofs.end())
				{
					temp_keypoints.push_back(keypoint);
				}
			}
			else if (it_1 == free_dofs.end())
			{
                temp_keypoints.push_back(keypoint);
			}
		}
		return temp_keypoints;
    } // delete_free_dof_keypoints()

    std::vector<Components::Point*> stabilize::order_keypoints_singular(Components::Point* point, unsigned int dof, std::vector<Components::Point*> keypoints)
    {
        stabilize::update_singular_values();
		std::map<std::pair<Components::Point*, unsigned int>, double>::iterator it_1; // singular_values
        std::map<double, Components::Point*, std::greater<double> > key_singulars;
		std::map<double, Components::Point*>::iterator it_2; //key_singulars
		std::vector<Components::Point*> constrained_keypoints;

		for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			std::pair<Components::Point*, unsigned int> key_dof = std::make_pair(keypoint, dof);
			it_1 = singular_values.find(key_dof);
			if (it_1 == singular_values.end())
				constrained_keypoints.push_back(keypoint);
			else
				key_singulars[it_1->second] = keypoint;
		}
		keypoints.clear();
		for (unsigned int i = 0; i < constrained_keypoints.size(); i++)
		{
			keypoints.push_back(constrained_keypoints[i]);
		}
		for(it_2 = key_singulars.begin(); it_2 != key_singulars.end(); it_2++)
		{
			keypoints.push_back(it_2->second);
		}
		return keypoints;
    } // order_keypoints_singular();

    std::vector<Components::Point*> stabilize::order_keypoints_internal(Components::Point* point, std::vector<Components::Point*> keypoints)
    {
        std::vector<Components::Point*> internal_keypoints;
		std::vector<Components::Point*> external_keypoints;

		for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			if (stabilize::find_rectangle(point, keypoint) == true)
				internal_keypoints.push_back(keypoint);
			else
				external_keypoints.push_back(keypoint);
		}
		keypoints.clear();
		keypoints = external_keypoints;
		keypoints.insert(keypoints.begin(), internal_keypoints.begin(), internal_keypoints.end());
		return keypoints;
    } // order_keypoints_internal()

    std::vector<Components::Point*> stabilize::order_keypoints_enveloppe(Components::Point* point, std::vector<Components::Point*> keypoints)
    {
        std::vector<Components::Point*> inside_keypoints;
		std::vector<Components::Point*> enveloppe_keypoints;

		for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			Components::Point* keypoint = keypoints[i];
			if (stabilize::find_rectangle(point, keypoint) == true && stabilize::get_rectangle(point, keypoint)->get_surface_count() == 1)
				enveloppe_keypoints.push_back(keypoint);
			else
				inside_keypoints.push_back(keypoint);
		}
		keypoints.clear();
		keypoints = inside_keypoints;
		keypoints.insert(keypoints.begin(), enveloppe_keypoints.begin(), enveloppe_keypoints.end());
		return keypoints;
    } // order_keypoints_enveloppe()

	std::vector<Components::Point*> stabilize::delete_used_keypoints_beam(Components::Point* point, std::vector<Components::Point*> keypoints)
	{
        std::vector<Components::Point*> temp_keypoints;
		for (unsigned int i = 0; i < keypoints.size(); i++)
        {
            Components::Point* keypoint = keypoints[i];
            std::pair<Components::Point*, Components::Point*> pair_1 = std::make_pair(point, keypoint);
            std::pair<Components::Point*, Components::Point*> pair_2 = std::make_pair(keypoint, point);
            if (std::find(added_beams.begin(), added_beams.end(), pair_1) == added_beams.end() &&
				std::find(added_beams.begin(), added_beams.end(), pair_2) == added_beams.end())
			{
				temp_keypoints.push_back(keypoint);
			}

        }
        return temp_keypoints;
	} // delete_used_keypoints_beam()

	std::vector<Components::Point*> stabilize::delete_unzoned_keypoints_beam(Components::Point* point, std::vector<Components::Point*> keypoints)
	{
        std::vector<Components::Point*> temp_keypoints;
		std::map<Components::Point*, spatial_design::Geometry::Vertex*>::iterator it_1; // point_vertex

		it_1 = point_vertex.find(point);
		spatial_design::Geometry::Vertex* v1 = it_1->second;
		for (unsigned int i = 0; i < keypoints.size(); i++)
        {
			it_1 = point_vertex.find(keypoints[i]);
			spatial_design::Geometry::Vertex* v2 = it_1->second;
		    for (unsigned int j = 0; j < m_CF->get_line_count(); j++)
		    {
				if (m_CF->get_line(j)->get_zoned() == true)
				{
					spatial_design::Geometry::Vertex* v3 = m_CF->get_line(j)->get_vertex_ptr(0);
					spatial_design::Geometry::Vertex* v4 = m_CF->get_line(j)->get_vertex_ptr(1);
					if ((v1 == v3 && v2 == v4) || (v1 == v4 && v2 == v3))
					{
						temp_keypoints.push_back(keypoints[i]);
						break;
					}
				}
			}
        }
        return temp_keypoints;
	} // delete_unzoned_keypoints_beam()

	std::vector<Components::Point*> stabilize::search_keypoints_truss(std::pair<Components::Point*, unsigned int> point_dof)
	{
		std::vector<Components::Point*> keypoints;
		std::vector<Components::Point*> keypoints_1;
		std::vector<Components::Point*> keypoints_2;
		unsigned int dof = point_dof.second;

		switch (dof)
        {
        case 0: // uX: keypoints in xz- and xy-planes
			keypoints_1 = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
            keypoints_2 = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
			keypoints = keypoints_2;
			keypoints.insert(keypoints.begin(), keypoints_1.begin(), keypoints_1.end());
            break;
        case 1: // uY: keypoints in yz- and xy-planes
			keypoints_1 = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
	        keypoints_2 = stabilize::get_keypoints_truss(point_dof, 'x', 'y');
			keypoints = keypoints_2;
			keypoints.insert(keypoints.begin(), keypoints_1.begin(), keypoints_1.end());
            break;
        case 2: // uZ: keypoints in xz- and yz-planes
			keypoints_1 = stabilize::get_keypoints_truss(point_dof, 'x', 'z');
	        keypoints_2 = stabilize::get_keypoints_truss(point_dof, 'y', 'z');
			keypoints = keypoints_2;
			keypoints.insert(keypoints.begin(), keypoints_1.begin(), keypoints_1.end());
            break;
        case 3: // rX (not implemented yet)

            break;
        case 4: // rY (not implemented yet)

            break;
        case 5: // rZ (not implemented yet)

            break;
        }
		return keypoints;
	} // search_keypoints_truss()

	std::vector<Components::Point*> stabilize::search_keypoints_beam(std::pair<Components::Point*, unsigned int> point_dof)
	{
		std::vector<Components::Point*> keypoints;
		std::vector<Components::Point*> keypoints_1;
		std::vector<Components::Point*> keypoints_2;
		unsigned int dof = point_dof.second;

		switch (dof)
        {
        case 0: // uX: keypoints on y- and z-axis
            keypoints_1 = stabilize::get_keypoints_beam(point_dof, 'y');
            keypoints_2 = stabilize::get_keypoints_beam(point_dof, 'z');
			keypoints = keypoints_2;
            keypoints.insert(keypoints.begin(), keypoints_1.begin(), keypoints_1.end());
            break;
        case 1: // uY: keypoints on x- and z-axis
            keypoints_1 = stabilize::get_keypoints_beam(point_dof, 'x');
            keypoints_2 = stabilize::get_keypoints_beam(point_dof, 'z');
			keypoints = keypoints_2;
            keypoints.insert(keypoints.begin(), keypoints_1.begin(), keypoints_1.end());
            break;
        case 2: // uZ: keypoints on x- and y-axis
            keypoints_1 = stabilize::get_keypoints_beam(point_dof, 'x');
            keypoints_2 = stabilize::get_keypoints_beam(point_dof, 'y');
			keypoints = keypoints_2;
            keypoints.insert(keypoints.begin(), keypoints_1.begin(), keypoints_1.end());
            break;
        case 3: // rX (not implemented yet)

            break;
        case 4: // rY (not implemented yet)

            break;
        case 5: // rZ (not implemented yet)

            break;
        }
		return keypoints;
	} // search_keypoints_beam()

    std::vector<Components::Point*> stabilize::get_keypoints_truss(std::pair<Components::Point*, unsigned int> free_dof, char dir_1, char dir_2)
    {
		std::map<Components::Point*, std::vector<unsigned int> >::iterator it_1; // free_dofs
        std::map<Components::Point*, std::vector<unsigned int> >::iterator it_2; // points_grid
        std::map<std::vector<unsigned int>, Components::Point*>::iterator it_3; // grid_points
        std::vector<unsigned int> dof_grid; // grid location of free DOF
        std::vector<vector<vector<coord*>>>::iterator it_x; // grid-location x
        std::vector<vector<coord*>>::iterator it_y; // grid-location y
        std::vector<coord*>::iterator it_z; // grid-location z
        std::vector<unsigned int> key_grid; // grid location of potential keypoint
		std::vector<Components::Point*> keypoints;
        bool found = false; // triggers loop-break when keypoint is found
        it_2 = points_grid.find(free_dof.first);
        dof_grid = it_2->second;
        if ((dir_1 == 'x' && dir_2 == 'z') || (dir_1 == 'z' && dir_2 == 'x'))
        {
            // xz: keypoint 1.
            found = false;
            for (it_x = grid.begin() + dof_grid[0] - 1 ; it_x >= grid.begin() ; it_x--)
            {
                it_y = it_x->begin() + dof_grid[1];
                for (it_z = it_y->begin() + dof_grid[2] - 1 ; it_z >= it_y->begin() ; it_z--)
                {
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }

            // xz: keypoint 2.
            found = false;
            for (it_x = grid.begin() + dof_grid[0] - 1 ; it_x >= grid.begin() ; it_x--)
            {
                it_y = it_x->begin() + dof_grid[1];
                for (it_z = it_y->begin() + dof_grid[2] + 1 ; it_z < it_y->end() ; it_z++)
                {
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }

            // xz: keypoint 3.
            found = false;
            for (it_x = grid.begin() + dof_grid[0] + 1 ; it_x < grid.end() ; it_x++)
            {
                it_y = it_x->begin() + dof_grid[1];
                for (it_z = it_y->begin() + dof_grid[2] - 1 ; it_z >= it_y->begin() ; it_z--)
                {
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }

            // xz: keypoint 4.
            found = false;
            for (it_x = grid.begin() + dof_grid[0] + 1 ; it_x < grid.end() ; it_x++)
            {
                it_y = it_x->begin() + dof_grid[1];
                for (it_z = it_y->begin() + dof_grid[2] + 1 ; it_z < it_y->end() ; it_z++)
                {
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                   it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }
        } // xz-plane

        else if ((dir_1 == 'y' && dir_2 == 'z') || (dir_1 == 'z' && dir_2 == 'y'))
        {
            // yz: keypoint 1.
            found = false;
            it_x = grid.begin() + dof_grid[0];
            for (it_y = it_x->begin() + dof_grid[1] - 1 ; it_y >= it_x->begin() ; it_y--)
            {
                for (it_z = it_y->begin() + dof_grid[2] - 1 ; it_z >= it_y->begin() ; it_z--)
                {
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }

            // yz: keypoint 2.
            found = false;
            it_x = grid.begin() + dof_grid[0];
            for (it_y = it_x->begin() + dof_grid[1] - 1 ; it_y >= it_x->begin() ; it_y--)
            {
                for (it_z = it_y->begin() + dof_grid[2] + 1 ; it_z < it_y->end() ; it_z++)
                {
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }

            // yz: keypoint 3.
            found = false;
            it_x = grid.begin() + dof_grid[0];
            for (it_y = it_x->begin() + dof_grid[1] + 1 ; it_y < it_x->end() ; it_y++)
            {
                for (it_z = it_y->begin() + dof_grid[2] - 1 ; it_z >= it_y->begin() ; it_z--)
                {
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }

            // yz: keypoint 4.
            found = false;
            it_x = grid.begin() + dof_grid[0];
            for (it_y = it_x->begin() + dof_grid[1] + 1 ; it_y < it_x->end() ; it_y++)
            {
                for (it_z = it_y->begin() + dof_grid[2] + 1 ; it_z < it_y->end() ; it_z++)
                {
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }
        } // yz-plane

        else if ((dir_1 == 'x' && dir_2 == 'y') || (dir_1 == 'y' && dir_2 == 'x'))
        {
			// xy: keypoint 1.
            found = false;
            for (it_x = grid.begin() + dof_grid[0] - 1 ; it_x >= grid.begin() ; it_x--)
            {
                for (it_y = it_x->begin() + dof_grid[1] - 1 ; it_y >= it_x->begin() - 1 ; it_y--)
                {
                    it_z = it_y->begin() + dof_grid[2];
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }

            // xy: keypoint 2.
            found = false;
            for (it_x = grid.begin() + dof_grid[0] - 1 ; it_x >= grid.begin() ; it_x--)
            {
                for (it_y = it_x->begin() + dof_grid[1] + 1 ; it_y < it_x->end() ; it_y++)
                {
                    it_z = it_y->begin() + dof_grid[2];
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }

            // xy: keypoint 3.
            found = false;
            for (it_x = grid.begin() + dof_grid[0] + 1 ; it_x < grid.end() ; it_x++)
            {
                for (it_y = it_x->begin() + dof_grid[1] - 1 ; it_y >= it_x->begin() -1 ; it_y--)
                {
                    it_z = it_y->begin() + dof_grid[2];
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }

            // xy: keypoint 4.
            found = false;
            for (it_x = grid.begin() + dof_grid[0] + 1 ; it_x < grid.end() ; it_x++)
            {
                for (it_y = it_x->begin() + dof_grid[1] + 1 ; it_y < it_x->end() ; it_y++)
                {
                    it_z = it_y->begin() + dof_grid[2];
                    key_grid.push_back(std::distance(grid.begin(), it_x));
                    key_grid.push_back(std::distance(it_x->begin(), it_y));
                    key_grid.push_back(std::distance(it_y->begin(), it_z));

                    it_3 = grid_points.find(key_grid);
                    if (it_3 != grid_points.end())
                    {
                        keypoints.push_back(it_3->second);
                        key_grid.clear();
                        found = true;
                        break;
                    }
                    key_grid.clear();
                }
                if (found == true)
                    break;
            }
        } // xy-plane
		return keypoints;
    } // get_keypoints_truss()

	std::vector<Components::Point*> stabilize::get_keypoints_beam(std::pair<Components::Point*, unsigned int> free_dof, char dir_1)
    {
        std::map<Components::Point*, std::vector<unsigned int> >::iterator it_1; // free_dofs
        std::map<Components::Point*, std::vector<unsigned int> >::iterator it_2; // points_grid
        std::map<std::vector<unsigned int>, Components::Point*>::iterator it_3; // grid_points
        std::vector<unsigned int> dof_grid; // grid location of free DOF
        std::vector<vector<vector<coord*>>>::iterator it_x; // grid-location x
        std::vector<vector<coord*>>::iterator it_y; // grid-location y
        std::vector<coord*>::iterator it_z; // grid-location z
        std::vector<unsigned int> key_grid; // grid location of potential keypoint
		std::vector<Components::Point*> keypoints;
        bool found; // triggers loop-break when keypoint is found

        it_2 = points_grid.find(free_dof.first);
        dof_grid = it_2->second;

        if (dir_1 == 'x')
        {
            // x: keypoint 1.
            found = false;
            for (it_x = grid.begin() + dof_grid[0] - 1 ; it_x >= grid.begin() ; it_x--)
            {
                it_y = it_x->begin() + dof_grid[1];
				it_z = it_y->begin() + dof_grid[2];

                key_grid.push_back(std::distance(grid.begin(), it_x));
                key_grid.push_back(std::distance(it_x->begin(), it_y));
                key_grid.push_back(std::distance(it_y->begin(), it_z));

                it_3 = grid_points.find(key_grid);
                if (it_3 != grid_points.end())
                {
                    keypoints.push_back(it_3->second);
                    key_grid.clear();
                    found = true;
                }
                key_grid.clear();

                if (found == true)
                    break;
            }

            // x: keypoint 2.
            found = false;
            for (it_x = grid.begin() + dof_grid[0] + 1 ; it_x < grid.end() ; it_x++)
            {
                it_y = it_x->begin() + dof_grid[1];
				it_z = it_y->begin() + dof_grid[2];

                key_grid.push_back(std::distance(grid.begin(), it_x));
                key_grid.push_back(std::distance(it_x->begin(), it_y));
                key_grid.push_back(std::distance(it_y->begin(), it_z));

                it_3 = grid_points.find(key_grid);
                if (it_3 != grid_points.end())
                {
                    keypoints.push_back(it_3->second);
                    key_grid.clear();
                    found = true;
                }
                key_grid.clear();

                if (found == true)
                    break;
            }
        } // x-axis

        else if (dir_1 == 'y')
        {
            // y: keypoint 1.
            found = false;
            it_x = grid.begin() + dof_grid[0];
			for (it_y = it_x->begin() + dof_grid[1] - 1 ; it_y >= it_x->begin() ; it_y--)
            {
				it_z = it_y->begin() + dof_grid[2];

                key_grid.push_back(std::distance(grid.begin(), it_x));
                key_grid.push_back(std::distance(it_x->begin(), it_y));
                key_grid.push_back(std::distance(it_y->begin(), it_z));

                it_3 = grid_points.find(key_grid);
                if (it_3 != grid_points.end())
                {
                    keypoints.push_back(it_3->second);
                    key_grid.clear();
                    found = true;
                }
                key_grid.clear();

                if (found == true)
                    break;
            }

            // y: keypoint 2.
            found = false;
            it_x = grid.begin() + dof_grid[0];
			for (it_y = it_x->begin() + dof_grid[1] + 1 ; it_y < it_x->end() ; it_y++)
            {
				it_z = it_y->begin() + dof_grid[2];

                key_grid.push_back(std::distance(grid.begin(), it_x));
                key_grid.push_back(std::distance(it_x->begin(), it_y));
                key_grid.push_back(std::distance(it_y->begin(), it_z));

                it_3 = grid_points.find(key_grid);
                if (it_3 != grid_points.end())
                {
                    keypoints.push_back(it_3->second);
                    key_grid.clear();
                    found = true;
                }
                key_grid.clear();

                if (found == true)
                    break;
            }
        } // y-axis

        else if (dir_1 == 'z')
        {
            // z: keypoint 1.
            found = false;
            it_x = grid.begin() + dof_grid[0];
			it_y = it_x->begin() + dof_grid[1];
			for (it_z = it_y->begin() + dof_grid[2] - 1 ; it_z >= it_y->begin() ; it_z--)
            {
                key_grid.push_back(std::distance(grid.begin(), it_x));
                key_grid.push_back(std::distance(it_x->begin(), it_y));
                key_grid.push_back(std::distance(it_y->begin(), it_z));

                it_3 = grid_points.find(key_grid);
                if (it_3 != grid_points.end())
                {
                    keypoints.push_back(it_3->second);
                    key_grid.clear();
                    found = true;
                }
                key_grid.clear();

                if (found == true)
                    break;
            }

            // z: keypoint 2.
            found = false;
            it_x = grid.begin() + dof_grid[0];
			it_y = it_x->begin() + dof_grid[1];
			for (it_z = it_y->begin() + dof_grid[2] + 1 ; it_z < it_y->end() ; it_z++)
            {
                key_grid.push_back(std::distance(grid.begin(), it_x));
                key_grid.push_back(std::distance(it_x->begin(), it_y));
                key_grid.push_back(std::distance(it_y->begin(), it_z));

                it_3 = grid_points.find(key_grid);
                if (it_3 != grid_points.end())
                {
                    keypoints.push_back(it_3->second);
                    key_grid.clear();
                    found = true;
                }
                key_grid.clear();

                if (found == true)
                    break;
            }
        } // z-axis

		return keypoints;
    } // get_keypoints_beam()

	void stabilize::add_truss(std::pair<Components::Point*, Components::Point*> dof_key)
	{
		if (stabilize::find_rectangle(dof_key.first, dof_key.second) == true)
		{
			spatial_design::Geometry::Rectangle* temp_rectangle = stabilize::get_rectangle(dof_key.first, dof_key.second);

			if (temp_rectangle->get_surface_count() == 0 && temp_rectangle->get_horizontal() == false)
			{
				std::map<spatial_design::Geometry::Rectangle*, std::vector<Components::Point*> >::iterator it_1; // rectangle_points
				it_1 = rectangle_points.find(temp_rectangle);
				std::vector<Components::Point*> temp = it_1->second;
				Components::Point* bottom_left = temp[0];
				Components::Point* top_left = temp[1];
				Components::Point* top_right = temp[2];
				Components::Point* bottom_right = temp[3];
				double x_mid = temp_rectangle->get_center_vertex_ptr()->get_coords()(0);
				double y_mid = temp_rectangle->get_center_vertex_ptr()->get_coords()(1);
				double z_mid = temp_rectangle->get_center_vertex_ptr()->get_coords()(2);
				for (unsigned int i = 0; i< temp.size(); i++)
				{
					if (temp[i]->get_coords()[2] < z_mid)
					{
						if (temp[i]->get_coords()[0] < x_mid || temp[i]->get_coords()[1] < y_mid)
							bottom_left = temp[i];
						else
							bottom_right = temp[i];
					}
					else if (temp[i]->get_coords()[2] > z_mid)
					{
						if (temp[i]->get_coords()[0] < x_mid || temp[i]->get_coords()[1] < y_mid)
							top_left = temp[i];
						else
							top_right = temp[i];
					}
				}
				stabilize::add_beam(bottom_left, top_left);
				stabilize::add_beam(top_left, top_right);
				stabilize::add_beam(top_right, bottom_right);
			}
			else
			{
				//std::cout << "Adding truss between points (" << dof_key.first->get_coords()[0] << "," << dof_key.first->get_coords()[1] << "," << 				dof_key.first->get_coords()[2]
				//		<< ") and (" << dof_key.second->get_coords()[0] << "," << dof_key.second->get_coords()[1] << "," << dof_key.second->get_coords()[2] << ")" << std::endl;

				Truss_Props props = m_SD->m_truss_props[0];
				m_SD->m_components.push_back( new Components::Truss(props.m_E, props.m_A, dof_key.first, dof_key.second));
			    m_SD->m_components.back()->set_mesh_switch(false);
			}
			temp_rectangle->make_structural();
		}
		else
		{
			//std::cout << "Adding truss between points (" << dof_key.first->get_coords()[0] << "," << dof_key.first->get_coords()[1] << "," << 				dof_key.first->get_coords()[2]
			//		<< ") and (" << dof_key.second->get_coords()[0] << "," << dof_key.second->get_coords()[1] << "," << dof_key.second->get_coords()[2] << ")" << std::endl;

			Truss_Props props = m_SD->m_truss_props[0];
			m_SD->m_components.push_back( new Components::Truss(props.m_E, props.m_A, dof_key.first, dof_key.second));
		    m_SD->m_components.back()->set_mesh_switch(false);
		}
	} // add_truss()

	void stabilize::add_truss(spatial_design::Geometry::Rectangle* temp_rectangle)
	{
		std::map<spatial_design::Geometry::Rectangle*, std::vector<Components::Point*> >::iterator it_1; // rectangle_points
		it_1 = rectangle_points.find(temp_rectangle);
		std::vector<Components::Point*> temp = it_1->second;
		Components::Point* bottom_left = temp[0];
		//Components::Point* top_left = temp[1];
		Components::Point* top_right = temp[2];
		//Components::Point* bottom_right = temp[3];
		double x_mid = temp_rectangle->get_center_vertex_ptr()->get_coords()(0);
		double y_mid = temp_rectangle->get_center_vertex_ptr()->get_coords()(1);
		double z_mid = temp_rectangle->get_center_vertex_ptr()->get_coords()(2);
		for (unsigned int i = 0; i< temp.size(); i++)
		{
			if (temp[i]->get_coords()[2] < z_mid)
			{
				if (temp[i]->get_coords()[0] < x_mid || temp[i]->get_coords()[1] < y_mid)
					bottom_left = temp[i];
			}
			else if (temp[i]->get_coords()[2] > z_mid)
			{
				if (temp[i]->get_coords()[0] > x_mid || temp[i]->get_coords()[1] > y_mid)
					top_right = temp[i];
			}
		}
		std::pair<Components::Point*, Components::Point*> temp_pair = std::make_pair(bottom_left, top_right);
		stabilize::add_truss(temp_pair);
	} // add_truss()

	void stabilize::add_beams(Components::Point* point, std::vector<Components::Point*> keypoints)
	{
		//std::cout << "Adding beam(s) around point (" << point->get_coords()[0] << "," << point->get_coords()[1] << "," << point->get_coords()[2] << ")" << std::endl;

		for (unsigned int i = 0; i < keypoints.size(); i++)
        {
            for (unsigned int j = 0; j < m_SD->m_components.size(); j++)
            {
                if (m_SD->m_components[j]->is_truss() == true)
                {
                    if (m_SD->m_components[j]->find_points(point, keypoints[i]) == true)
                    {
                        //m_SD->m_components.erase(m_SD->m_components.begin() + i);
                        std::rotate(m_SD->m_components.begin() + j, m_SD->m_components.begin() + j + 1, m_SD->m_components.end());
                        m_SD->m_components.pop_back();
                    }
                }
            }

            Beam_Props props = m_SD->m_beam_props[0];
            m_SD->m_components.push_back(new Components::Beam(props.m_b, props.m_h, props.m_E, props.m_v, point, keypoints[i]));
            m_SD->m_components.back()->set_mesh_switch(false);
			std::pair<Components::Point*, Components::Point*> temp_pair;
			temp_pair = std::make_pair(point, keypoints[i]);
			added_beams.push_back(temp_pair);
			temp_pair = std::make_pair(keypoints[i], point);
			added_beams.push_back(temp_pair);
        }
		//stabilize::delete_superfluous_truss(point, keypoints);
	} // add_beams()

	void stabilize::add_beam(Components::Point* p1, Components::Point* p2)
	{
        for (unsigned int j = 0; j < m_SD->m_components.size(); j++)
        {
            if (m_SD->m_components[j]->is_truss() == true)
            {
                if (m_SD->m_components[j]->find_points(p1, p2) == true)
                {
                    //m_SD->m_components.erase(m_SD->m_components.begin() + i);
                    std::rotate(m_SD->m_components.begin() + j, m_SD->m_components.begin() + j + 1, m_SD->m_components.end());
                    m_SD->m_components.pop_back();
                }
            }
        }

        Beam_Props props = m_SD->m_beam_props[0];
        m_SD->m_components.push_back(new Components::Beam(props.m_b, props.m_h, props.m_E, props.m_v, p1, p2));
        m_SD->m_components.back()->set_mesh_switch(false);
		std::pair<Components::Point*, Components::Point*> temp_pair;
		temp_pair = std::make_pair(p1, p2);
		added_beams.push_back(temp_pair);
		temp_pair = std::make_pair(p2, p1);
		added_beams.push_back(temp_pair);
	} // add_beam()

	void stabilize::delete_superfluous_truss(Components::Point* point, std::vector<Components::Point*> keypoints)
	{
		for (unsigned int i = 0; i < keypoints.size(); i++)
		{
			for (unsigned int j = 0; j < keypoints.size(); j++)
			{
				if (j != i && stabilize::find_rectangle(keypoints[i], keypoints[j]) == true)
				{
		            spatial_design::Geometry::Rectangle* temp_rectangle = stabilize::get_rectangle(keypoints[i], keypoints[j]);
		            if (temp_rectangle->is_structural() == true)
						stabilize::delete_truss(temp_rectangle);
					else
						temp_rectangle->make_structural();
				}
			}
		}
	} // delete_superfluous_truss()

	void stabilize::delete_superfluous_trusses()
	{
		for (unsigned int i = 0; i < m_CF->get_rectangle_count(); i++)
		{
			spatial_design::Geometry::Rectangle* temp_rectangle = m_CF->get_rectangle(i);
			if (temp_rectangle->is_structural() == true)
			{
				std::map<spatial_design::Geometry::Rectangle*, std::vector<Components::Point*> >::iterator it_1; // rectangle_points
				it_1 = rectangle_points.find(temp_rectangle);
				std::vector<Components::Point*> temp = it_1->second;
				std::vector<std::pair<Components::Point*, Components::Point*> > temp_pairs;
				std::pair<Components::Point*, Components::Point*> temp_pair;
				temp_pair = std::make_pair(temp[0], temp[1]); temp_pairs.push_back(temp_pair);
				temp_pair = std::make_pair(temp[1], temp[2]); temp_pairs.push_back(temp_pair);
				temp_pair = std::make_pair(temp[2], temp[3]); temp_pairs.push_back(temp_pair);
				temp_pair = std::make_pair(temp[3], temp[0]); temp_pairs.push_back(temp_pair);

				unsigned int beam_count = 0;
				for (unsigned int j = 0; j < 4; j++)
				{
					if (std::find(added_beams.begin(), added_beams.end(), temp_pairs[j]) != added_beams.end())
						beam_count++;
				}
				if (beam_count > 1)
					stabilize::delete_truss(temp_rectangle);
			}
		}
	} // delete_superfluous_trusses()

	void stabilize::delete_truss(spatial_design::Geometry::Rectangle* rectangle)
	{
		std::map<spatial_design::Geometry::Rectangle*, std::vector<Components::Point*> >::iterator it_1; // rectangle_points
		it_1 = rectangle_points.find(rectangle);
		std::vector<Components::Point*> points = it_1->second;
		for (unsigned int i = 0; i < m_SD->m_components.size(); i++)
        {
            if (m_SD->m_components[i]->is_truss() == true)
            {
                if (m_SD->m_components[i]->find_points(points[0], points[2]) == true)
                {
                    //m_SD->m_components.erase(m_SD->m_components.begin() + i);
                    std::rotate(m_SD->m_components.begin() + i, m_SD->m_components.begin() + i + 1, m_SD->m_components.end());
                    m_SD->m_components.pop_back();
                }
                else if (m_SD->m_components[i]->find_points(points[1], points[3]) == true)
                {
                    //m_SD->m_components.erase(m_SD->m_components.begin() + i);
                    std::rotate(m_SD->m_components.begin() + i, m_SD->m_components.begin() + i + 1, m_SD->m_components.end());
                    m_SD->m_components.pop_back();
                }
            }
        }
	} // delete_truss()

    void stabilize::relate_points_geometry()
    {
        unsigned int vertex_count = m_CF->get_vertex_count();
        for (unsigned int i = 0; i < m_points.size(); i++) // relate structural points and vertices
        {
            for (unsigned int j = 0; j < vertex_count; j++)
            {
                spatial_design::Geometry::Vertex* temp_vertex = m_CF->get_vertex(j);
                if (m_points[i]->get_coords() == temp_vertex->get_coords())
                {
                    point_vertex[m_points[i]] = temp_vertex;
					vertex_point[temp_vertex] = m_points[i];
                    break;
                }
            }
        }
		std::map<spatial_design::Geometry::Vertex*, Components::Point*>::iterator it_1; // vertex_point
		unsigned int rectangle_count = m_CF->get_rectangle_count();
		for (unsigned int i = 0; i < rectangle_count; i++)
		{
			spatial_design::Geometry::Rectangle* temp_rectangle = m_CF->get_rectangle(i);
			for (int j = 0; j < 4; j++)
			{
				spatial_design::Geometry::Vertex* temp_vertex = temp_rectangle->get_vertex_ptr(j);
				it_1 = vertex_point.find(temp_vertex);
				rectangle_points[temp_rectangle].push_back(it_1->second); // relate each rectangle to a vector of 4 structural points
				point_rectangles[it_1->second].push_back(temp_rectangle); // relate each point to a vector of rectangles
			}
		}
    } // relate_points_geometry()

	bool stabilize::find_rectangle(Components::Point* p1, Components::Point* p2)
	{
        std::map<Components::Point*, std::vector<spatial_design::Geometry::Rectangle*> >::iterator it_1; // point_rectangles
		std::map<spatial_design::Geometry::Rectangle*, std::vector<Components::Point*> >::iterator it_2; // rectangle_points
		spatial_design::Geometry::Rectangle* temp_rectangle;
		std::vector<Components::Point*> temp_points;
		bool found = false;

		it_1 = point_rectangles.find(p1);
		for (unsigned int i = 0; i < it_1->second.size(); i++)
		{
			temp_rectangle = it_1->second[i];
			it_2 = rectangle_points.find(temp_rectangle);
			temp_points = it_2->second;
			if (std::find(temp_points.begin(), temp_points.end(), p2) != temp_points.end())
			{
				found = true;
				break;
			}
		}
		return found;
	} // find_rectangle()

	spatial_design::Geometry::Rectangle* stabilize::get_rectangle(Components::Point* p1, Components::Point* p2)
	{
        std::map<Components::Point*, std::vector<spatial_design::Geometry::Rectangle*> >::iterator it_1; // point_rectangles
		std::map<spatial_design::Geometry::Rectangle*, std::vector<Components::Point*> >::iterator it_2; // rectangle_points
		spatial_design::Geometry::Rectangle* temp_rectangle = m_CF->get_rectangle(0); // just to initialize
		std::vector<Components::Point*> temp_points;

		it_1 = point_rectangles.find(p1);
		for (unsigned int i = 0; i < it_1->second.size(); i++)
		{
			temp_rectangle = it_1->second[i];
			it_2 = rectangle_points.find(temp_rectangle);
			temp_points = it_2->second;
			if (std::find(temp_points.begin(), temp_points.end(), p2) != temp_points.end())
			{
				break;
			}
		}
		return temp_rectangle;
	} // get_rectangle()

	void stabilize::update_singular_values()
	{
		singular_values = m_SD->get_points_singular_values();
	} // update_singular_values()

	std::vector<Components::Point*> stabilize::get_zone_points(spatial_design::zoning::zone* zone)
	{
		std::vector<spatial_design::Geometry::Vertex*> zone_vertices = zone->get_vertices();
		std::vector<Components::Point*> zone_points;
		std::map<spatial_design::Geometry::Vertex*, Components::Point*>::iterator it_1; // vertex_point
		for (unsigned int i = 0; i < zone_vertices.size(); i++)
		{
			it_1 = vertex_point.find(zone_vertices[i]);
			zone_points.push_back(it_1->second);
		}
		return zone_points;
	} // get_zone_points()

	void stabilize::delete_primary_zone_dofs()
	{
		std::map<Components::Point*, std::vector<unsigned int> >::iterator it_1; // free_dofs

		for (unsigned int i = 0; i < zones.size(); i++)
		{
			if (zones[i].primary == true && zones[i].stable == false)
			{
				for (unsigned int j = 0; j < zones[i].points.size(); j++)
				{
					it_1 = free_dofs.find(zones[i].points[j]);
					if (it_1 != free_dofs.end())
						free_dofs.erase(it_1);
				}
			}
		}
	} // delete_primary_zone_dofs()

	void stabilize::get_floor_grid()
	{
		for (unsigned int i = 0; i < floor_coords.size(); i++)
		{
			for (unsigned int j = 0; j < m_points.size(); j++)
			{
				if (m_points[j]->get_coords()[2] == floor_coords[i])
				{
					floor_grid[floors[i]] = points_grid[m_points[j]][2];
					grid_floor[points_grid[m_points[j]][2]] = floors[i];
					break;
				}
			}
		}
	} // get_floor_grid()

	void stabilize::get_floor_point()
	{
		for (unsigned int i = 0; i < floor_coords.size(); i++)
		{
			for (unsigned int j = 0; j < m_points.size(); j++)
			{
				if (m_points[j]->get_coords()[2] == floor_coords[i])
				{
					floor_point[floors[i]] = m_points[j];
					point_floor[m_points[j]] = floors[i];
				}
			}
		}
	} // get_floor_points()

	void stabilize::check_floating_zones()
	{
		std::map<std::vector<unsigned int>, Components::Point*>::iterator it_1; // grid_points
		Components::Point* point;
		std::vector<unsigned int> point_grid;

		for (unsigned int i = 0; i < zones.size(); i++)
		{
			if (zones[i].zone->get_floor() > 0)
			{
                for (unsigned int j = 0; j < 4; j++)
                {
                    unsigned int l = 0;
                    unsigned int m = 0;
                    unsigned int n = 0;
                    switch (j)
                    {
                        case 0: // north
						{
							m = zones[i].y_size-1;
							for (unsigned int k = 0; k < zones[i].x_size; k++)
							{
								l = k;
			                    point_grid = {l, m, n};
			                    it_1 = zones[i].grid_points.find(point_grid);
			                    point = it_1->second;
								for (unsigned int p = 0; p < zones.size(); p++)
								{
									if (p != i )//&& zones[i].floor == zones[p].floor_above)
									{
										if (std::find(zones[p].points.begin(), zones[p].points.end(), point) != zones[p].points.end())
										{
											zones[i].north_floating = false;
											break;
										}
										if (std::find(zones[p].points.begin(), zones[p].points.end(), point) == zones[p].points.end())
											zones[i].north_floating = true;
									}
								}
								if (zones[i].north_floating == true)
									break;
							}
							break;
						}
                        case 1: // east
						{
							l = zones[i].x_size-1;
							for (unsigned int k = 0; k < zones[i].y_size; k++)
							{
								m = k;
			                    point_grid = {l, m, n};
			                    it_1 = zones[i].grid_points.find(point_grid);
			                    point = it_1->second;
								for (unsigned int p = 0; p < zones.size(); p++)
								{
									if (p != i )//&& zones[i].floor == zones[p].floor_above)
									{
										if (std::find(zones[p].points.begin(), zones[p].points.end(), point) != zones[p].points.end())
										{
											zones[i].east_floating = false;
											break;
										}
										if (std::find(zones[p].points.begin(), zones[p].points.end(), point) == zones[p].points.end())
											zones[i].east_floating = true;
									}
								}
								if (zones[i].east_floating == true)
									break;
							}
							break;
						}
                        case 2: // south
						{
							m = 0;
							for (unsigned int k = 0; k < zones[i].x_size; k++)
							{
								l = k;
			                    point_grid = {l, m, n};
			                    it_1 = zones[i].grid_points.find(point_grid);
			                    point = it_1->second;
								for (unsigned int p = 0; p < zones.size(); p++)
								{
									if (p != i )//&& zones[i].floor == zones[p].floor_above)
									{
										if (std::find(zones[p].points.begin(), zones[p].points.end(), point) != zones[p].points.end())
										{
											zones[i].south_floating = false;
											break;
										}
										if (std::find(zones[p].points.begin(), zones[p].points.end(), point) == zones[p].points.end())
											zones[i].south_floating = true;
									}
								}
								if (zones[i].south_floating == true)
									break;
							}
							break;
						}
                        case 3: // west
						{
							l = 0;
							for (unsigned int k = 0; k < zones[i].y_size; k++)
							{
								m = k;
			                    point_grid = {l, m, n};
			                    it_1 = zones[i].grid_points.find(point_grid);
			                    point = it_1->second;
								for (unsigned int p = 0; p < zones.size(); p++)
								{
									if (p != i )//&& zones[i].floor == zones[p].floor_above)
									{
										if (std::find(zones[p].points.begin(), zones[p].points.end(), point) != zones[p].points.end())
										{
											zones[i].west_floating = false;
											break;
										}
										if (std::find(zones[p].points.begin(), zones[p].points.end(), point) == zones[p].points.end())
											zones[i].west_floating = true;
									}
								}
								if (zones[i].west_floating == true)
									break;
							}
							break;
						}
                    }
                }
			}
		}
	} // check_floating_zones()

    void stabilize::show_free_dofs()
    {
        std::map<Components::Point*, std::vector<unsigned int> >::iterator it;

        if (free_dofs.size() == 0)
            std::cout << std::endl << "No free DOF's found" << std::endl;
        else
        {
            std::cout << std::endl << "Points with free DOF's:" << std::endl;
            for (it = free_dofs.begin() ; it != free_dofs.end() ; it++)
            {
                std::cout << std::endl << "Coords: (" << it->first->get_coords()[0] << ", " << it->first->get_coords()[1] << ", " <<
                it->first->get_coords()[2] << ")" << std::endl;

                std::cout << "Free DOF's: ";

                for (unsigned int i = 0 ; i < it->second.size() ; i++)
                {
                    switch (it->second[i])
                    {
                    case 0:
                        std::cout << "uX";
                        if (i < it->second.size()-1)
                            std::cout << ", ";
                        break;
                    case 1:
                        std::cout << "uY";
                        if (i < it->second.size()-1)
                            std::cout << ", ";
                        break;
                    case 2:
                        std::cout << "uZ";
                        if (i < it->second.size()-1)
                            std::cout << ", ";
                        break;
                    case 3:
                        std::cout << "rX";
                        if (i < it->second.size()-1)
                            std::cout << ", ";
                        break;
                    case 4:
                        std::cout << "rY";
                        if (i < it->second.size()-1)
                            std::cout << ", ";
                        break;
                    case 5:
                        std::cout << "rZ";
                        break;
                    }
                }
                std::cout << std::endl;
            }
        }
    } // show_free_dofs()

    void stabilize::show_singulars()
    {
        std::map<std::pair<Components::Point*, unsigned int>, double>::iterator it; // singular_values
        for (it = singular_values.begin(); it != singular_values.end(); it++)
        {
            std::cout << "point (" << it->first.first->get_coords()[0] << ", " << it->first.first->get_coords()[1] << ", "
            << it->first.first->get_coords()[2] << ") dof: " << it->first.second << " singular value: " << it->second << std::endl;
        }
    } // show_singulars()

} // namespace stabilization
} // namespace structural_design
} // namespace bso

#endif //STABILIZE_HPP
