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

import datetime
import math
from carebt_kb.owlready2_kb import OwlReady2Kb


class TestOwlReady2Kb_TestOwl():

    def test_initialize(self):
        # setup
        kb = OwlReady2Kb('src/carebt_ros2/carebt_kb/test/data/test.owl')
        assert len(list(kb.test.individuals())) == 0

    def test_create_individuals(self):
        # setup
        kb = OwlReady2Kb('src/carebt_ros2/carebt_kb/test/data/test.owl')
        assert len(list(kb.test.individuals())) == 0

        # subtest
        subtest = kb.create({'type': 'test.Subtest', 'id': 1})

        # subtest list
        subtests = []
        subtests.append(kb.create({'type': 'test.Subtest', 'id': 2}))
        assert isinstance(subtests[0], str)
        assert subtests[0] == 'test.subtest2'
        subtests.append(kb.create({'type': 'test.Subtest', 'id': 3}))
        assert isinstance(subtests[1], str)
        assert subtests[1] == 'test.subtest3'
        subtests.append(kb.create({'type': 'test.Subtest', 'id': 4}))
        assert isinstance(subtests[2], str)
        assert subtests[2] == 'test.subtest4'

        t1 = kb.create({'type': 'test.Test',
                        # functional
                        'test_int': 1,
                        'test_bool': True,
                        'test_float': 1.23,
                        'test_str': 'Hello World',
                        'test_datetime': datetime.datetime(2022, 2, 22, 21, 55, 59, 123456),
                        'test_date': datetime.date(2022, 2, 22),
                        'test_time': datetime.time(21, 55, 59, 123456),
                        'test_normstr': 'This is a normstring.',
                        # locstr
                        'test_locstr': [('German', 'de'), ('English', 'en'), ('Spanish', 'es')],
                        # non-functional
                        'test_int_list': [1,2,3],
                        'test_bool_list': [True, False, True],
                        'test_float_list': [1.11, 2.22, 3.33],
                        'test_str_list': ['first', 'second', 'third'],
                        'test_datetime_list': [datetime.datetime(2022, 2, 21, 11, 22, 33, 123456), datetime.datetime(2022, 2, 22, 21, 55, 59, 123456)],
                        'test_date_list': [datetime.date(2022, 2, 21), datetime.date(2022, 2, 22)],
                        'test_time_list': [datetime.time(11, 22, 33, 123456), datetime.time(21, 55, 59, 123456)],
                        'test_normstr_list': ['This is a normstring.', 'This is another String'],
                        'has_one_subtest': subtest,
                        'has_subtest': subtests
                    })

        r1 = kb.search({'type': 'test.Test', 'test_int': 1})
        # functional
        assert isinstance(r1[0]['test_int'], int)
        assert r1[0]['test_int'] == 1
        assert isinstance(r1[0]['test_bool'], bool)
        assert r1[0]['test_bool'] is True
        assert isinstance(r1[0]['test_float'], float)
        assert math.isclose(r1[0]['test_float'], 1.23)
        assert isinstance(r1[0]['test_str'], str)
        assert r1[0]['test_str'] == 'Hello World'
        assert isinstance(r1[0]['test_datetime'], datetime.datetime)
        assert r1[0]['test_datetime'] == datetime.datetime(2022, 2, 22, 21, 55, 59, 123456)
        assert isinstance(r1[0]['test_date'], datetime.date)
        assert r1[0]['test_date'] == datetime.date(2022, 2, 22)
        assert isinstance(r1[0]['test_time'], datetime.time)
        assert r1[0]['test_time'] == datetime.time(21, 55, 59, 123456)
        assert isinstance(r1[0]['test_normstr'], str)  # ???
        assert r1[0]['test_normstr'] == 'This is a normstring.'
        # locstr
        assert isinstance(r1[0]['test_locstr'], list)
        assert isinstance(r1[0]['test_locstr'][0], tuple)
        assert isinstance(r1[0]['test_locstr'][0][0], str)
        assert isinstance(r1[0]['test_locstr'][0][1], str)
        # non-functional
        assert len(r1[0]['test_int_list']) == 3
        assert r1[0]['test_int_list'][0] == 1
        assert len(r1[0]['test_bool_list']) == 3
        assert r1[0]['test_bool_list'][0] is True
        assert len(r1[0]['test_float_list']) == 3
        assert math.isclose(r1[0]['test_float_list'][0], 1.11)
        assert len(r1[0]['test_str_list']) == 3
        assert r1[0]['test_str_list'][0] == 'first'
        assert len(r1[0]['test_datetime_list']) == 2
        assert r1[0]['test_datetime_list'][0] == datetime.datetime(2022, 2, 21, 11, 22, 33, 123456)
        assert len(r1[0]['test_date_list']) == 2
        assert r1[0]['test_date_list'][0] == datetime.date(2022, 2, 21)
        assert len(r1[0]['test_time_list']) == 2
        assert r1[0]['test_time_list'][0] == datetime.time(11, 22, 33, 123456)
        assert len(r1[0]['test_normstr_list']) == 2
        assert isinstance(r1[0]['has_one_subtest'], str)
        assert r1[0]['has_one_subtest'] == 'test.subtest1'
        assert len(r1[0]['has_subtest']) == 3
        assert isinstance(r1[0]['has_subtest'][0], str)
        assert r1[0]['has_subtest'][0] == 'test.subtest2'
        assert r1[0]['has_subtest'][1] == 'test.subtest3'
        assert r1[0]['has_subtest'][2] == 'test.subtest4'

    def test_create_test_id_1(self):
        # setup
        kb = OwlReady2Kb('src/carebt_ros2/carebt_kb/test/data/test.owl')
        assert len(list(kb.test.individuals())) == 0

        # Test
        t = kb.create({'type': 'test.Test', 'test_int': 1})
        r1 = kb.search({'type': 'test.Test', 'test_int': 1})

    def test_read_unknown_slot(self):
        # setup
        kb = OwlReady2Kb('src/carebt_ros2/carebt_kb/test/data/test.owl')
        assert len(list(kb.test.individuals())) == 0

        # Test
        t = kb.create({'type': 'test.Test', 'test_int': 1})
        r1 = kb.search({'type': 'test.Test', 'xxx': 1})
        assert len(r1) == 0

    def test_update(self):
        # setup
        kb = OwlReady2Kb('src/carebt_ros2/carebt_kb/test/data/test.owl')
        assert len(list(kb.test.individuals())) == 0

        # Test
        kb.create({'type': 'test.Test', 'test_int': 1})
        r1 = kb.search({'type': 'test.Test'})
        assert len(r1) == 1

        # subtest
        subtest = kb.create({'type': 'test.Subtest', 'id': 1})

        # subtest list
        subtests = []
        subtests.append(kb.create({'type': 'test.Subtest', 'id': 2}))
        assert isinstance(subtests[0], str)
        assert subtests[0] == 'test.subtest2'
        subtests.append(kb.create({'type': 'test.Subtest', 'id': 3}))
        assert isinstance(subtests[1], str)
        assert subtests[1] == 'test.subtest3'
        subtests.append(kb.create({'type': 'test.Subtest', 'id': 4}))
        assert isinstance(subtests[2], str)
        assert subtests[2] == 'test.subtest4'

        kb.update({'type': 'test.Test', 'test_int': 1},
                  {'test_int': 11,
                   'test_bool': False,
                   'test_float': 11.234,
                   'test_str': 'Hello another World',
                   'test_datetime': datetime.datetime(2022, 1, 24, 11, 33, 22, 654321),
                   'test_date': datetime.date(2020, 3, 30),
                   'test_time': datetime.time(23, 59, 59, 112233),
                   'test_normstr': 'This is a normstring.',
                   # locstr
                   'test_locstr': [('Hallo', 'de'), ('Hello', 'en'), ('Hola', 'es')],
                   # non-functional
                   'test_int_list': [9,8,7],
                   'test_bool_list': [False, True, False],
                   'test_float_list': [9.99, 8.88, 7.77],
                   'test_str_list': ['one', 'two', 'three'],
                   'test_datetime_list': [datetime.datetime(2020, 1, 11, 22, 23, 24, 242322),
                                          datetime.datetime(2020, 11, 1, 23, 22, 21, 212223),
                                          datetime.datetime(2020, 12, 1, 22, 22, 22, 222222)],
                   'test_date_list': [datetime.date(2022, 1, 10),],
                   'test_time_list': [datetime.time(1, 2, 3, 123123), datetime.time(4, 5, 6, 456456)],
                   'test_normstr_list': ['Test string 1', 'Test string 2'],
                   'has_one_subtest': subtest,
                   'has_subtest': subtests
                  })
        r1 = kb.search({'type': 'test.Test'})
        assert len(r1) == 1
        # functional
        assert isinstance(r1[0]['test_int'], int)
        assert r1[0]['test_int'] == 11
        assert isinstance(r1[0]['test_bool'], bool)
        assert r1[0]['test_bool'] is False
        assert isinstance(r1[0]['test_float'], float)
        assert math.isclose(r1[0]['test_float'], 11.234)
        assert isinstance(r1[0]['test_str'], str)
        assert r1[0]['test_str'] == 'Hello another World'
        assert isinstance(r1[0]['test_datetime'], datetime.datetime)
        assert r1[0]['test_datetime'] == datetime.datetime(2022, 1, 24, 11, 33, 22, 654321)
        assert isinstance(r1[0]['test_date'], datetime.date)
        assert r1[0]['test_date'] == datetime.date(2020, 3, 30)
        assert isinstance(r1[0]['test_time'], datetime.time)
        assert r1[0]['test_time'] == datetime.time(23, 59, 59, 112233)
        # locstr
        assert isinstance(r1[0]['test_locstr'], list)
        assert isinstance(r1[0]['test_locstr'][0], tuple)
        assert isinstance(r1[0]['test_locstr'][0][0], str)
        assert isinstance(r1[0]['test_locstr'][0][1], str)
        assert r1[0]['test_locstr'][0][0] == 'Hallo'
        assert r1[0]['test_locstr'][0][1] == 'de'
        assert r1[0]['test_locstr'][1][0] == 'Hello'
        assert r1[0]['test_locstr'][1][1] == 'en'
        assert r1[0]['test_locstr'][2][0] == 'Hola'
        assert r1[0]['test_locstr'][2][1] == 'es'
        # non-functional
        assert len(r1[0]['test_int_list']) == 3
        assert r1[0]['test_int_list'][0] == 9
        assert r1[0]['test_int_list'][1] == 8
        assert r1[0]['test_int_list'][2] == 7
        assert len(r1[0]['test_bool_list']) == 3
        assert r1[0]['test_bool_list'][0] is False
        assert len(r1[0]['test_float_list']) == 3
        assert math.isclose(r1[0]['test_float_list'][0], 9.99)
        assert math.isclose(r1[0]['test_float_list'][1], 8.88)
        assert math.isclose(r1[0]['test_float_list'][2], 7.77)
        assert len(r1[0]['test_str_list']) == 3
        assert r1[0]['test_str_list'][0] == 'one'
        assert r1[0]['test_str_list'][1] == 'two'
        assert r1[0]['test_str_list'][2] == 'three'
        assert len(r1[0]['test_datetime_list']) == 3
        assert r1[0]['test_datetime_list'][0] == datetime.datetime(2020, 1, 11, 22, 23, 24, 242322)
        assert r1[0]['test_datetime_list'][1] == datetime.datetime(2020, 11, 1, 23, 22, 21, 212223)
        assert r1[0]['test_datetime_list'][2] == datetime.datetime(2020, 12, 1, 22, 22, 22, 222222)
        assert len(r1[0]['test_date_list']) == 1
        assert r1[0]['test_date_list'][0] == datetime.date(2022, 1, 10)
        assert len(r1[0]['test_time_list']) == 2
        assert r1[0]['test_time_list'][0] == datetime.time(1, 2, 3, 123123)
        assert r1[0]['test_time_list'][1] == datetime.time(4, 5, 6, 456456)
        assert len(r1[0]['test_normstr_list']) == 2
        assert r1[0]['test_normstr_list'][0] == 'Test string 1'
        assert r1[0]['test_normstr_list'][1] == 'Test string 2'
        assert isinstance(r1[0]['has_one_subtest'], str)
        assert r1[0]['has_one_subtest'] == 'test.subtest1'
        assert len(r1[0]['has_subtest']) == 3
        assert isinstance(r1[0]['has_subtest'][0], str)
        assert r1[0]['has_subtest'][0] == 'test.subtest2'
        assert r1[0]['has_subtest'][1] == 'test.subtest3'
        assert r1[0]['has_subtest'][2] == 'test.subtest4'
