import multiprocessing
import timeit
import time
import signal
import os


class MyProcess(multiprocessing.Process):

    def __init__(self, process_id, name, work_queue,return_queue, queue_lock, num_processes, master_pid,
                 fail_safe_planner):
        multiprocessing.Process.__init__(self)
        self.id = process_id
        self.name = name
        self.work_queue = work_queue
        self.return_queue = return_queue
        self.queue_lock = queue_lock
        self.num_processes = num_processes
        self.fail_safe_planner = fail_safe_planner
        self.master_pid = master_pid
        signal.signal(signal.SIGUSR1, self.handler_slave)

    def run(self):
        process_data(self.id, self.work_queue, self.return_queue, self.queue_lock,
                     self.num_processes, self.master_pid, self.fail_safe_planner)

    def join(self, *args):
        multiprocessing.Process.join(self, *args)
        return

    def handler_slave(self, signum, frame):
        print("Exiting")
        os._exit(0)


def process_data(process_id, w_q, r_q, queue_lock, num_processes, master_pid, fail_safe_planner):
    trajectories_for_idxs = []
    queue_lock.acquire()
    print("Process %d started" % process_id)
    while not w_q.empty():
        time_idx = w_q.get()
        print("Process %d got job: %d" % (process_id, time_idx))
        if time_idx == -1:
            r_q.put(trajectories_for_idxs)
            queue_lock.release()
            return
        queue_lock.release()
        init_x = fail_safe_planner.desired_trajectory.state_list[time_idx].position[0]
        init_y = fail_safe_planner.desired_trajectory.state_list[time_idx].position[1]
        init_velocity = fail_safe_planner.desired_trajectory.state_list[time_idx].velocity
        init_acceleration = fail_safe_planner.desired_trajectory.state_list[time_idx].acceleration
        start_time = timeit.default_timer()
        trajectory = fail_safe_planner.fail_safe_trajectory_from_state(init_x, init_y, init_velocity, init_acceleration,
                                                                       time_idx)
        if trajectory is None:
            print("Process %d did not find a trajectory for time idx: %d in : %f s" % (process_id, time_idx,
                                                                                       timeit.default_timer() -
                                                                                       start_time))
        else:
            print("Process %d found a trajectory for time idx: %d in : %f s" % (process_id, time_idx,
                                                                                timeit.default_timer() - start_time))
        trajectories_for_idxs.append((time_idx, trajectory))
        if trajectory is not None:
            queue_lock.acquire()
            r_q.put(trajectories_for_idxs)
            queue_lock.release()
            os.kill(master_pid, signal.SIGUSR1)
            return
        queue_lock.acquire()
    print("Process %d is exiting" % process_id)
    r_q.put(trajectories_for_idxs)
    queue_lock.release()
    os.kill(master_pid, 31)
    return
