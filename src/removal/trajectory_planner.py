import os
import argparse
import timeit

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.common.solution_writer import CommonRoadSolutionWriter
from commonroad.scenario.trajectory import Trajectory

from removal.plot_trajectories import plot_sequence, create_video
from removal.trajectory_tree import TrajectoryTree
from removal.fail_safe_planner import FailSafePlanner


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Solves planning task for a given scenario')
    parser.add_argument('filename', metavar='f', type=str, help='Path to .xml scenario file')
    parser.add_argument('output_dir', metavar='o', type=str, help='Directory for solution xml and video if -v is set')
    parser.add_argument('-fail_safe', '-fs', action='store_true', dest='b_fail_safe', help='Plan fail safe trajectory')
    parser.add_argument('-video', '-v', action='store_true',
                        dest='b_video', help='Export .mp4 video file from solution')

    args = parser.parse_args()

    scenario, planning_problem_set = CommonRoadFileReader(args.filename).open()

    for planning_problem in planning_problem_set.planning_problem_dict.values():
        start = timeit.default_timer()
        search_tree = TrajectoryTree(planning_problem, scenario)
        optimal_trajectory = search_tree.uniform_cost_search()

        if optimal_trajectory is None:
            raise Exception('Found no feasible trajectory to goal region for planning problem %d'
                            % planning_problem.planning_problem_id)
        print('Found trajectory for planning problem id %d: %f s'
              % (planning_problem.planning_problem_id, timeit.default_timer() - start))

        # planning_task.planning_problems[0].add_solution(optimal_trajectory, scenario.dt)
        state_list = []
        for opt_trajectory in optimal_trajectory:
            state_list.extend(opt_trajectory.state_list[:-1])

        final_trajectory = Trajectory(0, state_list)

        # add solution to scenario and write it to xml
        f = os.path.basename(args.filename)
        f_without_ext = os.path.splitext(f)[0]

        CSW = CommonRoadSolutionWriter(args.output_dir,
                                       scenario.benchmark_id, scenario.dt)
        CSW.add_solution_trajectory(final_trajectory, planning_problem.planning_problem_id)
        CSW.write_to_file(overwrite=True)
        print("Wrote xml file to %s" % args.output_dir)

        if args.b_video:
            out_path = os.path.join(args.output_dir, f_without_ext + '_' + str(planning_problem.planning_problem_id)
                                    + '.mp4')
            create_video(out_path, scenario, final_trajectory, planning_problem_set)
            print("Created video %s" % out_path)

        if args.b_fail_safe:
            print("plan fail_safe")

            start = timeit.default_timer()

            fs_planer = FailSafePlanner(scenario, final_trajectory)

            fail_safe_trajectory_list = fs_planer.fail_safe_trajectory()
            if fail_safe_trajectory_list:
                print('Found fail-safe trajectory for planning problem id %d: %f s'
                      % (planning_problem.planning_problem_id, timeit.default_timer() - start))

                plot_sequence(args.output_dir, scenario, planning_problem_set,
                              final_trajectory, fail_safe_trajectory_list)
            else:
                print('No fail-safe trajectory found for planning problem id %d'
                      % planning_problem.planning_problem_id)
