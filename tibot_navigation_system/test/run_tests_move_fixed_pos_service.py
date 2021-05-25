#! /usr/bin/env python

import rosunit
import tests_move_fixed_pos_service

# rosunit
print("Running all test...")
rosunit.unitrun('tibot_navigation_system.test', 'tests_move_fixed_pos_service', 'tests_move_fixed_pos_service.MyTestSuite')
