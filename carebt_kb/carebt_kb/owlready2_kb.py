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


class OwlReady2Kb():

    OWL_KEYWORDS = ['iri', 'is_a', 'name', 'namespace', 'storid']

    def __init__(self, filename: str, sync_to_file: bool = False):
        self.__filename = filename
        self.__sync_on = sync_to_file
        self.__onto = filename.split('/')[-1].split('.')[0]
        self.__onto_inferrences = self.__onto + '_inferrences'

        # load ontology from file
        exec(f'self.{self.__onto} = get_ontology(filename).load()')

        # create ontology for inferrences (by reasoner pellet)
        base_iri = eval(f'self.{self.__onto}.base_iri')
        base = base_iri.split('/')[-1].split('.')[0]
        self.__base_inferrences_iri = base_iri.replace(base, base+'_inferrences')
        exec(f'self.{self.__onto_inferrences} = get_ontology(\'{self.__base_inferrences_iri}\')')

    # PRIVATE

    def __get_items(self, filter):
        items = []
        if len(filter) > 0:
            try:
                if 'type' in filter:
                    filter['type'] = eval(f'self.{filter["type"]}')
                items = eval(f'self.{self.__onto}.search(**filter)')
            except Exception as ex:
                pass
        return items

    def __onto_to_dict(self, clazz: ThingClass):
        dict_str: str = '{'
        dict_str += f'\'iri\': \'{clazz.iri}\', '
        dict_str += f'\'is_a\': ['
        for c in clazz.is_a:
            dict_str += f'\'{str(c)}\', '
        dict_str += '], '
        # iterate over properties
        for prop in clazz.get_properties():
            is_functional = prop.is_functional_for(clazz.__class__)
            key_type = prop.range[0]
            value = eval(f"self.{clazz}.{prop.name}")
            # owlready2.entity.ThingClass
            if key_type.__class__ is owlready2.entity.ThingClass: 
                if is_functional:
                    dict_str += f'\'{prop.name}\': \'{value}\', '
                else:
                    list_str = '['
                    for v in value:
                        list_str += f'\'{v}\', '
                    list_str += ']'
                    dict_str += f'\'{prop.name}\': {list_str}, '
            # owlready2.util.normstr
            elif issubclass(key_type, owlready2.util.normstr): # TODO normstr ???
                if is_functional:
                    dict_str += f'\'{prop.name}\': \'{value}\', '
                else:
                    dict_str += f'\'{prop.name}\': {value}, '
            # owlready2.util.locstr
            elif issubclass(key_type, owlready2.util.locstr):
                list_str = '['
                for v in value:
                    list_str += f'{v}, '
                list_str += ']'
                dict_str += f'\'{prop.name}\': {list_str}, '
            # int, float
            elif issubclass(key_type, (int, float)):
                dict_str += f'\'{prop.name}\': {value}, '
            # bool
            elif issubclass(key_type, (bool)):
                dict_str += f'\'{prop.name}\': {value}, '
            # str
            elif issubclass(key_type, (str)):
                if prop.name.endswith('_rosstr'):
                    if is_functional:
                        dict_str += f'\'{prop.name}\': {value}, '
                    else:
                        v_list = []
                        for v in value:
                            v_list.append(eval(v))
                        dict_str += f'\'{prop.name}\': {v_list}, '
                else:
                    if is_functional:
                        dict_str += f'\'{prop.name}\': \'{value}\', '
                    else:
                        dict_str += f'\'{prop.name}\': {value}, '
            # datetime.datetime
            elif issubclass(key_type, (datetime.datetime)):
                if is_functional:
                    dict_str += f'\'{prop.name}\': datetime.datetime.fromisoformat("{value}"), '
                else:
                    list_str = '['
                    for v in value:
                        list_str += f'datetime.datetime.fromisoformat("{v}"), '
                    list_str += ']'
                    dict_str += f'\'{prop.name}\': {list_str}, '
            # datetime.date
            elif issubclass(key_type, (datetime.date)):
                if is_functional:
                    dict_str += f'\'{prop.name}\': datetime.date.fromisoformat("{value}"), '
                else:
                    list_str = '['
                    for v in value:
                        list_str += f'datetime.date.fromisoformat("{v}"), '
                    list_str += ']'
                    dict_str += f'\'{prop.name}\': {list_str}, '
            # datetime.time
            elif issubclass(key_type, (datetime.time)):
                if is_functional:
                    dict_str += f'\'{prop.name}\': datetime.time.fromisoformat("{value}"), '
                else:
                    list_str = '['
                    for v in value:
                        list_str += f'datetime.time.fromisoformat("{v}"), '
                    list_str += ']'
                    dict_str += f'\'{prop.name}\': {list_str}, '            
            # unknown
            else:
                print('unknown type')
        dict_str += '}'
        #print(f'__onto_to_dict - eval(s): {dict_str}')
        return eval(dict_str)

    def __dict_to_typed_dict(self, frame):
        typed_dict = {}
        typed_dict['type'] = frame['type']
        for key in frame.keys():
            if key == 'type':
                continue
            if eval(f'self.{self.__onto}.{key}') == None:
                print(f'The key {key} is not part of the ontology.')
                continue
            if key not in self.OWL_KEYWORDS:
                range = eval(f'self.{self.__onto}.{key}.range')
                clazz = frame['type']
                clazz  # supress unused variable warning 
                is_functional = eval(f'self.{self.__onto}.{key}.is_functional_for(clazz)')
                key_type = range[0]
                # owlready2.entity.ThingClass
                if key_type.__class__ is owlready2.entity.ThingClass:
                    if is_functional:
                        typed_dict[key] = f'self.{frame[key]}'
                    else:
                        list_str = '['
                        for item in frame[key]:
                            list_str += f'self.{item}, '
                        list_str += ']'
                        typed_dict[key] = list_str
                # owlready2.util.normstr
                elif issubclass(key_type, owlready2.util.normstr):
                    if is_functional:
                        typed_dict[key] = f'"{frame[key]}"'
                    else:
                        typed_dict[key] = frame[key]
                # owlready2.util.locstr
                elif issubclass(key_type, owlready2.util.locstr):
                    inner_list = []
                    for item in frame[key]:
                        inner_list.append(locstr(item))
                    typed_dict[key] = inner_list
                # int, float, bool
                elif issubclass(key_type, (int, float, bool)):
                    typed_dict[key] = frame[key]
                # str
                elif issubclass(key_type, (str)):
                    # if the str is a ROS str
                    if key.endswith('_rosstr'):
                        if is_functional:
                            typed_dict[key] = f'"{frame[key]}"'
                        else:
                            str_value_list = []
                            for value in frame[key]:
                                str_value_list.append(f'{value}')
                            typed_dict[key] = str_value_list
                    # else it is a 'normal' str
                    else:
                        if is_functional:
                            typed_dict[key] = f'"{frame[key]}"'
                        else:
                            typed_dict[key] = frame[key]
                # datetime.datetime
                elif issubclass(key_type, (datetime.datetime)):
                    if is_functional:
                        typed_dict[key] = f'datetime.datetime.fromisoformat("{frame[key]}")'
                    else:
                        inner_list = []
                        for item in frame[key]:
                            inner_list.append(datetime.datetime.fromisoformat(f'{item}'))
                        typed_dict[key] = inner_list
                # datetime.date
                elif issubclass(key_type, (datetime.date)):
                    if is_functional:
                        typed_dict[key] = f'datetime.date.fromisoformat("{frame[key]}")'
                    else:
                        inner_list = []
                        for item in frame[key]:
                            inner_list.append(datetime.date.fromisoformat(f'{item}'))
                        typed_dict[key] = inner_list
                # datetime.time
                elif issubclass(key_type, (datetime.time)):
                    if is_functional:
                        typed_dict[key] = f'datetime.time.fromisoformat("{frame[key]}")'
                    else:
                        inner_list = []
                        for item in frame[key]:
                            inner_list.append(datetime.time.fromisoformat(f'{item}'))
                        typed_dict[key] = inner_list
                # unknown
                else:
                    print('unknown type')
            else:
                print(f'create - do not use key: {key}')
        return typed_dict

    def __update(self, item, update):
        typed_update = self.__dict_to_typed_dict(update)
        for k in typed_update.keys():
            if k in ['type']:
                continue
            if k not in self.OWL_KEYWORDS:
                try:
                    exec(f'self.{item}.{k} = {typed_update[k]}')
                except Exception as ex:
                    print(f'__update excpetion: {ex}') 
            else:
                print(f'update - do not use key: {k}')

    def __sync_to_file(self):
        if self.__sync_on:
            self.save()

    # PUBLIC

    def create(self, frame) -> str:
        if 'type' not in frame:
            print(f'To create a new item a type is required.')
            return None
        else:
            onto_str = f'self.{frame["type"]}()'
            # create item
            item = eval(onto_str)
            # update item
            self.__update(item, frame)
            self.__sync_to_file()
            return str(item)

    def search(self, filter):
        items = []
        for o in self.__get_items(filter):
            items.append(self.__onto_to_dict(o))
        return items

    def read(self, items):
        rtrn_items = []
        for i in items:
            o = eval(f'self.{i}')
            rtrn_items.append(self.__onto_to_dict(o))
        return rtrn_items
    
    def update(self, filter, update):
        for item in self.__get_items(filter):
            update['type'] = eval(f'self.{item}.__class__')
            self.__update(item, update)
        self.__sync_to_file()

    def delete(self, filter):
        for item in self.__get_items(filter):
            destroy_entity(item)
        self.__sync_to_file()

    def get_classes(self):
        clazzes = list(eval(f'self.{self.__onto}.classes()'))
        str_list = []
        for clazz in clazzes:
            str_list.append(str(clazz))
        return str_list

    def get_subclasses_of(self, class_str: str):
        if class_str == 'Thing':
            clazz = Thing
        else:
            clazz = eval(f'self.{class_str}')
        subclazzes = list(clazz.subclasses())
        str_list = []
        for clazz in subclazzes:
            str_list.append(str(clazz))
        return str_list
    def has_subclasses(self, class_str: str):
        return len(self.get_subclasses_of(class_str)) > 0

    def get_individuals_of(self, class_str: str):
        if class_str == 'Thing':
            clazz = Thing
        else:
            clazz = eval(f'self.{class_str}')
        instances = list(clazz.instances())
        str_list = []
        for instance in instances:
            str_list.append(str(instance))
        return str_list

    def is_individual_of(self, individual_str: str, class_str: str) -> bool:
        if class_str == 'Thing':
            return eval(f'isinstance(self.{individual_str}, Thing)')
        else:
            return eval(f'isinstance(self.{individual_str}, self.{class_str})')

    def trigger_reasoner(self, reasoner: str, debug: int = 0):
        if eval(f'self.{self.__onto_inferrences}.loaded'):
                eval(f'self.{self.__onto_inferrences}.destroy()')
                exec(f'self.{self.__onto_inferrences} = get_ontology(\'{self.__base_inferrences_iri}\')')
        with eval(f'self.{self.__onto_inferrences}'):
            if reasoner == 'hermit':
                sync_reasoner_hermit(debug = debug)
            elif reasoner == 'pellet':
                sync_reasoner_pellet(infer_property_values = True, infer_data_property_values = True, debug = debug)
            else:
                print('UNKNOWN REASONER')

    def save(self):
        eval(f'self.{self.__onto}.save({self.__filename}, format = "rdfxml")')
