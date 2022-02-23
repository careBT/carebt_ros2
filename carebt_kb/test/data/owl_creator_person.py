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


person = get_ontology('http://test.org/person.owl#')

with person:
    class Person(Thing): pass
    class first_name(Person >> str, FunctionalProperty): pass
    class age (Person >> int, FunctionalProperty): pass
    class size (Person >> float, FunctionalProperty): pass
    class weight (Person >> float, FunctionalProperty): pass
    class bmi (Person >> float, FunctionalProperty): pass

    class OldPerson(Person):
        equivalent_to = [
            Person & age.some(ConstrainedDatatype(int, min_inclusive = 65))
        ]

    class TallPerson(Person):
        equivalent_to = [
            Person & (size >= 1.8) # shortened notation
        ]

    class ObesePerson(Person):
        equivalent_to = [
            Person & (bmi >= 30.0)
        ]

    imp = Imp()
    imp.set_as_rule("Person(?person),\
                    weight(?person, ?weight),\
                    size(?person, ?size),\
                    multiply(?size2, ?size, ?size),\
                    divide(?bmi, ?weight, ?size2)\
                    -> bmi(?person, ?bmi)")


# test individuals
Person(first_name = 'Alice', age = 23, size = 1.65, weight = 51.0)
Person(first_name = 'Bob', age = 21, size = 1.8, weight = 95.0)
Person(first_name = 'Chuck', age = 28, size = 1.95, weight = 105.0)
Person(first_name = 'Dave', age = 16, size = 1.75, weight = 64.0)
Person(first_name = 'Eve', age = 23, size = 1.85, weight = 78.0)
Person(first_name = 'Frank', age = 23, size = 1.75, weight = 80.0)


## save
person.save('src/carebt_ros2/carebt_kb/test/data/person.owl', format = 'rdfxml')
