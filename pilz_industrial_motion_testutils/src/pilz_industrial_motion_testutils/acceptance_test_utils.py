#!/usr/bin/env python
# -*- coding: utf-8 -*-

#
# Copyright © 2018 Pilz GmbH & Co. KG
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Asks the user permission to start the test.
def _askPermission(test_name):
    s = raw_input('Perform ' + test_name + ' [(y)es, (n)o]?: ')
    if(s == "n"):
        print('\n\nSkip ' + test_name + '\n___TEST-END___\n')
        return 0
    print('\n\nStart ' + test_name + '\n')
    return 1

# Asks the user if the test was successful and (if given) displays
# a hint regarding the assessment of a successful test.
def _askSuccess(test_name, question=None):
    if (question != None):
        print('\nTest ' + test_name + ' successful?')
        print('Hint: \n' + question)

    s = raw_input('Test ' + test_name + ' successful [(y)es, (n)o]?: ')
    if(s == "n"):
        print('\nTest ' + test_name + ' failed!\n___TEST-END___\n')
        return 0
    print('Test ' + test_name + ' successful.\n___TEST-END___\n')
    return 1
