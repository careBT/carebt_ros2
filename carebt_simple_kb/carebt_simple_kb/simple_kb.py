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

import json
import sys
import threading
from uuid import uuid4 as get_uuid


class SimpleKb():

    def __init__(self, filename: str, sync_to_file: bool = False):
        self.__filename = filename
        self.__sync_to_file = sync_to_file
        self.__kb_dict = {}
        self.__lock = threading.Lock()

        try:
            f = open(filename)
            self.__kb_dict = json.load(f)
            f.close()
        except IOError:
            print(f"{filename} - File not accessible")

    # PRIVATE

    def __save(self, sync_to_file: bool):
        if(sync_to_file):
            json.dump(self.__kb_dict, open(self.__filename, 'w'))

    def __get_uuids(self, filter):
        uuids = []
        for uuid, e in self.__kb_dict.items():
            match = {k: e[k] for k in filter.keys() if k in e and filter[k] == e[k]}
            # if all keys match
            if(len(filter) == len(match)):
                uuids.append(uuid)
        return uuids

    # PUBLIC

    def create(self, frame) -> None:
        """
        Create the frame in the knowledge base.

        Parameters
        ----------
        frame: dict
            Frame to create
        """
        with self.__lock:
            self.__kb_dict[str(get_uuid())] = frame
            self.__save(self.__sync_to_file)

    def read(self, filter):
        """
        Read the frames matching the filter.

        Parameters
        ----------
        filter: dict
            Filter for the frames to search

        Returns
        -------
        [dict]
            A list of frames

        """
        frames = []
        with self.__lock:
            uuids = self.__get_uuids(filter)
            for uuid in uuids:
                frames.append(self.__kb_dict[uuid])
        return frames

    def update(self, filter, update) -> None:
        """
        Update the provided slots matching the filter.

        Update the provided slots matching the filter. If the filter does not
        match the frame is created.

        Parameters
        ----------
        filter: dict
            Filter for the frames to update
        update: dict
            The slots to update

        """
        with self.__lock:
            uuids = self.__get_uuids(filter)
            if(len(uuids) > 0):
                for uuid in uuids:
                    self.__kb_dict[uuid].update(update)
            else:
                # merge filter and update
                filter.update(update)
                self.__kb_dict[str(get_uuid())] = filter
            self.__save(self.__sync_to_file)

    def delete(self, filter):
        """
        Delete the frames matching the filter.

        Parameters
        ----------
        filter: dict
            Filter for the frames to update

        """
        with self.__lock:
            uuids = self.__get_uuids(filter)
            for uuid in uuids:
                del self.__kb_dict[uuid]
            self.__save(self.__sync_to_file)

    def print(self):
        """
        Print the complete knowledge base.

        """
        print(json.dumps(self.__kb_dict, sort_keys=True, indent=2))

    def size(self) -> int:
        """
        Return the size of the knowledge base in bytes.

        """
        return sys.getsizeof(self.__kb_dict)

    def count(self) -> int:
        """
        Return the number of frames in the knowledge base.

        """
        return len(self.__kb_dict)

    def save(self):
        """
        Save the knowledge base.

        """
        with self.__lock:
            self.__save(True)
            self.get_logger().info(f'kb saved to file {self.__filename} with {self.size_in_kb} bytes; '
                + f'{self.count()} entries')
