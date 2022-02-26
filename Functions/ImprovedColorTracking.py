#!/usr/bin/python3
# coding=utf8

import time
import concurrent.futures
import perception
import movement
import bus

def concurrent_perception(perception_inst,perception_bus,movement_bus,delay):
    while True:
        movement_data = movement_bus.read()
        perception_data = perception_inst.perceive(movement_data)
        perception_bus.write(perception_data)
        time.sleep(delay)
        if not perception_data['isRunning']:
            perception.shutdown()
            break

def concurrent_move(movement_inst,perception_bus,movement_bus,delay):
    while True:
        perception_data = perception_bus.read()
        movement_data = movement_inst.move(perception_data)
        movement_bus.write(movement_data)
        time.sleep(delay)
        if not movement_data['isRunning']:
            movement_inst.shutdown()
            break

def init_busses(perception_bus,movement_bus,perception_inst,movement_inst):
    """Put valid valid values on the busses before entering operation."""
    perception_bus.write(perception_inst.gather_data_to_send())
    movement_bus.write(movement_inst.gather_data_to_send())

def main():
    perception_inst = perception.Perception(perception_mode='tracking',target_color='red')
    movement_inst = movement.Movement()
    perception_bus = bus.Bus()
    movement_bus = bus.Bus()
    init_busses(perception_bus,movement_bus,perception_inst,movement_inst)
    # Run perception and movement class instances until keyboard break.
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        thread1 = executor.submit(concurrent_perception,perception_inst,perception_bus,movement_bus,0.01)
        thread2 = executor.submit(concurrent_move,movement_inst,perception_bus,movement_bus,0.01)
    thread1.result()
    thread2.result()

if __name__ == '__main__':
    main()
