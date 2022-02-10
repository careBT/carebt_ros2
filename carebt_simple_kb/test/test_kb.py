# Copyright 2021 Andreas Steck (steck.andi@gmail.com)
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

from carebt_simple_kb.carebt_simple_kb import Kb

class TestKb():

    def test_create_from_file(self):
        kb = Kb('src/carebt_ros2/carebt_simple_kb/test/data/memory.json')
        assert len(kb._Kb__kb) == 4

    def test_get_uuids(self):
        kb = Kb('src/carebt_ros2/carebt_simple_kb/test/data/memory.json')

        result = kb._Kb__get_uuids({'is-a': 'Person', 'name': 'Bob'})
        assert len(result) == 1
        assert result[0] == '7a84f3c5-9c60-491c-8dde-38e2572fc970'

        result = kb._Kb__get_uuids({'is-a': 'Person', 'name': 'Alice'})
        assert len(result) == 2

        result = kb._Kb__get_uuids({'is-a': 'Person', 'name': 'XXX'})
        assert len(result) == 0

        result = kb._Kb__get_uuids({'is-a': 'Person'})
        print(result)
        assert len(result) == 4

    def test_read(self):
        kb = Kb('src/carebt_ros2/carebt_simple_kb/test/data/memory.json')

        result = kb.read({'is-a': 'Person', 'name': 'Bob'})
        assert len(result) == 1
        assert len(result[0]) == 3
        assert result[0]['is-a'] == 'Person'
        assert result[0]['name'] == 'Bob'
        assert result[0]['age'] == 39

        result = kb.read({'is-a': 'Person', 'name': 'Alice'})
        assert len(result) == 2

        result = kb.read({'is-a': 'Person', 'name': 'XXX'})
        assert len(result) == 0   

        result = kb.read({'is-a': 'Person'})
        assert len(result) == 4   

     

    def test_create(self):
        kb = Kb('src/carebt_ros2/carebt_simple_kb/test/data/memory.json')

        kb.create({'is-a': 'Person', 'name': 'Dave', 'age': 23})

        assert len(kb._Kb__kb) == 5

        result = kb.read({'is-a': 'Person', 'name': 'Dave'})
        assert len(result) == 1
        assert len(result[0]) == 3
        assert result[0]['is-a'] == 'Person'
        assert result[0]['name'] == 'Dave'
        assert result[0]['age'] == 23
        
    def test_update_bob(self):
        kb = Kb('src/carebt_ros2/carebt_simple_kb/test/data/memory.json')

        kb.update({'is-a': 'Person', 'name': 'Bob'}, {'age': 55})

        assert len(kb._Kb__kb) == 4

        result = kb.read({'is-a': 'Person', 'name': 'Bob'})
        assert len(result) == 1
        assert len(result[0]) == 3
        assert result[0]['is-a'] == 'Person'
        assert result[0]['name'] == 'Bob'
        assert result[0]['age'] == 55

    def test_update_alice_both(self):
        kb = Kb('src/carebt_ros2/carebt_simple_kb/test/data/memory.json')

        kb.update({'is-a': 'Person', 'name': 'Alice'}, {'age': 99})

        assert len(kb._Kb__kb) == 4

        result = kb.read({'is-a': 'Person', 'name': 'Bob'})
        assert result[0]['age'] == 39

        result = kb.read({'is-a': 'Person', 'name': 'Alice'})
        assert result[0]['age'] == 99 
        result = kb.read({'is-a': 'Person', 'name': 'Alice'})
        assert result[0]['age'] == 99 

    def test_update_alice_5(self):
        kb = Kb('src/carebt_ros2/carebt_simple_kb/test/data/memory.json')

        kb.update({'is-a': 'Person', 'name': 'Alice', 'age': 5}, {'age': 99})

        assert len(kb._Kb__kb) == 4

        result = kb.read({'is-a': 'Person', 'name': 'Bob'})
        assert result[0]['age'] == 39

        result = kb.read({'is-a': 'Person', 'name': 'Alice', 'age': 16})
        assert len(result) == 1
        result = kb.read({'is-a': 'Person', 'name': 'Alice', 'age': 5})
        assert len(result) == 0
        result = kb.read({'is-a': 'Person', 'name': 'Alice', 'age': 99})
        assert len(result) == 1

    def test_show(self):
        kb = Kb('src/carebt_ros2/carebt_simple_kb/test/data/memory.json')
        print("KB entries")
        kb.show()