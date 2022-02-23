# Copyright 2022 Andreas Steck (steck.andi@gmail.com)
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

from owlready2 import *


test = get_ontology('http://test.org/test.owl#')

with test:
    class Test(Thing): pass
    class Subtest(Thing): pass
    
    # Task
    class test_int(Test >> int, FunctionalProperty): pass
    class test_bool(Test >> bool, FunctionalProperty): pass
    class test_float(Test >> float, FunctionalProperty): pass
    class test_str(Test >> str, FunctionalProperty): pass
    class test_datetime(Test >> datetime.datetime, FunctionalProperty): pass
    class test_date(Test >> datetime.date, FunctionalProperty): pass
    class test_time(Test >> datetime.time, FunctionalProperty): pass
    class test_normstr(Test >> owlready2.normstr, FunctionalProperty): pass
    
    class test_locstr(Test >> owlready2.locstr): pass

    class test_normstr_list(Test >> owlready2.normstr): pass
    class test_int_list(Test >> int): pass
    class test_bool_list(Test >> bool): pass
    class test_float_list(Test >> float): pass
    class test_str_list(Test >> str): pass
    class test_datetime_list(Test >> datetime.datetime): pass
    class test_date_list(Test >> datetime.date): pass
    class test_time_list(Test >> datetime.time): pass
    
    # SubTask
    class id(Subtest >> str, FunctionalProperty): pass
    
    AllDisjoint([Test, Subtest])

    # relations
    class has_one_subtest(Test >> Subtest, FunctionalProperty): pass
    class is_single_subtest_of(Subtest >> Test, FunctionalProperty):
        inverse = has_one_subtest

    class has_subtest(Test >> Subtest): pass
    class is_subtest_of(Subtest >> Test):
        inverse = has_subtest

## save
test.save('src/carebt_ros2/carebt_kb/test/data/test.owl', format = 'rdfxml')
