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

from carebt_msgs.srv import RequestResponse
import json
import rclpy
from rclpy.node import Node
from uuid import uuid4 as get_uuid

class Kb():

    def __init__(self, filename: str, sync_to_file: bool = False):
        self.__filename = filename
        self.__sync_to_file = sync_to_file
        self.__kb = {}

        try:
            f = open(filename)
            self.__kb = json.load(f)
            f.close()
        except IOError:
            print(f"{filename} - File not accessible")            

    def __save(self):
        if(self.__sync_to_file):
            json.dump(self.__kb, open(self.__filename, 'w'))    

    def __get_uuids(self, filter):
        uuids = []
        for uuid, e in self.__kb.items():
            match = {k: e[k] for k in filter.keys() if k in e and filter[k] == e[k]}
            # if all keys match
            if(len(filter) == len(match)):
                uuids.append(uuid)
        return uuids 


    # PUBLIC

    def create(self, entry):
        self.__kb[str(get_uuid())] = entry
        self.__save()

    def read(self, filter):
        entries = []
        uuids = self.__get_uuids(filter)
        for uuid in uuids:
            entries.append(self.__kb[uuid])
        return entries

    def update(self, filter, update):
        uuids = self.__get_uuids(filter)
        if(len(uuids) > 0):
            for uuid in uuids:
                for key in update:
                    self.__kb[uuid][key] = update[key]
        else:
            self.__kb[str(get_uuid())] = update
        self.__save()

    def delete(self, keys, entry):
        uuids = self.__get_uuids(keys, entry)
        for uuid in uuids:
            del self.__kb[uuid]
        self.__save()

    def show(self):
        for uuid, e in self.__kb.items():
            print(f'{uuid}: {e}')


class KbServer(Node):

    def __init__(self):
        super().__init__('carebt_simplekb')
        # TODO
        self.__kb = Kb('src/carebt_ros2/carebt_simplekb/test/data/memory.json')

        # create ports
        self.srv = self.create_service(RequestResponse, 'query', self.query_callback)

    def query_callback(self, request: RequestResponse.Request, response: RequestResponse.Response):
        self.get_logger().info(f'Incoming request: {request.operation} {request.filter} {request.data}')

        # create
        if(request.operation.upper() == 'CREATE'):
            data = json.loads(request.data)
            self.__kb.create(data)
            result = []
            response.response = json.dumps(result)
        # read
        elif(request.operation.upper() == 'READ'):
            filter = json.loads(request.filter)
            result = self.__kb.read(filter)
            response.response = json.dumps(result)
        # update
        elif(request.operation.upper() == 'UPDATE'):
            filter = json.loads(request.filter)
            update = json.loads(request.data)
            self.__kb.update(filter, update)
            result = self.__kb.read(filter)
            response.response = json.dumps(result)
        # delete         
        elif(request.operation.upper() == 'DELETE'):
            filter = json.loads(request.filter)
            self.__kb.delete(filter)
            result = []
            response.response = json.dumps(result)
        else:
            print(f'unsupported operation ({request.operation}), use: CREATE/READ/UPDATE/DELETE')

        return response


def main(args=None):
    rclpy.init(args=args)

    node = KbServer()

    rclpy.spin(node)
    
    node._client.shutdown('root', 'ros')
    rclpy.shutdown()


if __name__ == '__main__':
    main()