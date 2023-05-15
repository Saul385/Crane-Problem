///////////////////////////////////////////////////////////////////////////////
// cranes_algs.hpp
//
// Algorithms that solve the crane unloading problem.
//
// All of the sections for this project reside in this file.
//
// This file builds on crane_types.hpp, so you should familiarize yourself
// with that file before working on this file.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cassert>
#include <math.h>

#include "cranes_types.hpp"

namespace cranes
{

  // Solve the crane unloading problem for the given grid, using an exhaustive
  // optimization algorithm.
  //
  // This algorithm is expected to run in exponential time, so the grid's
  // width+height must be small enough to fit in a 64-bit int; this is enforced
  // with an assertion.
  //
  // The grid must be non-empty.
  path crane_unloading_exhaustive(const grid &setting)
  {
    assert(setting.rows() > 0 && setting.columns() > 0 && "Grid non empty");

    const size_t max_steps = setting.rows() + setting.columns() - 2;
    assert(max_steps < 64 && "Maximum path length is illegal.");

    path best(setting);
    path new_path(setting);

    std::vector<path> all_paths;
    all_paths.push_back(new_path);

    for (size_t steps = 0; steps <= max_steps; steps++)
    {
      if (best.final_row() + 1 == setting.rows() && best.final_column() + 1 == setting.columns())
        break;

      std::vector<path> new_paths;
      new_paths.swap(all_paths);

      while (!new_paths.empty())
      {
        path current_path = std::move(new_paths.back());
        new_paths.pop_back();

        if (current_path.final_row() + 1 == setting.rows() && current_path.final_column() + 1 == setting.columns())
        {
          if (best.total_cranes() < current_path.total_cranes())
          {
            best = std::move(current_path);
          }
        }
        else
        {
          if (current_path.is_step_valid(STEP_DIRECTION_EAST))
          {
            path next_path = current_path;
            next_path.add_step(STEP_DIRECTION_EAST);
            all_paths.push_back(std::move(next_path));
          }
          if (current_path.is_step_valid(STEP_DIRECTION_SOUTH))
          {
            path next_path = current_path;
            next_path.add_step(STEP_DIRECTION_SOUTH);
            all_paths.push_back(std::move(next_path));
          }
        }
      }
    }

    return best;
  }

  // Solve the crane unloading problem for the given grid, using a dynamic
  // programming algorithm.
  //
  // The grid must be non-empty.
  path crane_unloading_dyn_prog(const grid &setting)
  {

    path best(setting);
    assert(setting.rows() > 0);
    assert(setting.columns() > 0);
    int my_grid[setting.rows() + 1][setting.columns() + 1];
    cell_kind current_cell;
    for (int i = 0; i < setting.rows() + 1; i++)
    {
      for (int j = 0; j < setting.columns() + 1; j++)
      {
        if (i == 0 || j == 0)
        {
          my_grid[i][j] = 0;
        }
        else
        {
          if (i == 1 && j == 1)
            my_grid[i][j] = 1;
          else
            my_grid[i][j] = 0;
          current_cell = setting.get(i - 1, j - 1);
          if (current_cell == CELL_BUILDING)
          {
            my_grid[i][j] = -1;
          }
          else
          {
            if (current_cell == CELL_CRANE)
            {
              my_grid[i][j] += 1;
            }
            int max;
            if (my_grid[i - 1][j] > my_grid[i][j - 1])
            {
              max = my_grid[i - 1][j];
            }
            else
              max = my_grid[i][j - 1];
            my_grid[i][j] += max;
          }
        }
      }
    }
    const size_t max_steps = setting.rows() + setting.columns() - 2;
    int x = setting.rows();
    int y = setting.columns();
    std::vector<step_direction> directions;
    for (int i = 0; i < max_steps; i++)
    {
      if ((my_grid[x - 1][y] == -1) && (my_grid[x][y - 1] == -1))
      {
        y--;
        x--;
      }
      else if (my_grid[x][y] == -1)
      {
        y--;
        break;
      }
      else if ((my_grid[x - 1][y] >= my_grid[x][y - 1]) && x != 1)
      {
        directions.insert(directions.end(), STEP_DIRECTION_SOUTH);
        x--;
      }
      else if (y != 1)
      {
        directions.insert(directions.end(), STEP_DIRECTION_EAST);
        y--;
      }
    }

    int size = directions.size();
    for (int i = size; i > 0; i--)
    {
      step_direction current_direction = directions.back();
      if (best.is_step_valid(current_direction))
        best.add_step(current_direction);
      directions.pop_back();
    }
    return best;
  }
}