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

from carebt_kb.plugin_base import PluginBase
from functools import partial
import json
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading


class SimpleWebServer(BaseHTTPRequestHandler):

    def __init__(self, kb, *args, **kwargs):
        self.__kb = kb
        super().__init__(*args, **kwargs)

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        str_clazzes = self.__kb.get_classes()
        # start HTML
        self.wfile.write(bytes("<html>\
            <head>\
                <title>careBT web view</title>\
            <style>\
                table, th, td { border: 1px solid black; }\
                table {padding: 10px;}\
            </style>\
            </head><body>", "utf-8"))
        # iterate classes
        for str_clazz in str_clazzes:
            if not self.__kb.has_subclasses(str_clazz):
                self.wfile.write(bytes(f"<h3>{str_clazz}</h3>", "utf-8"))
                str_individuals = self.__kb.get_individuals_of(str_clazz)
                individuals = self.__kb.read(str_individuals)
                for individual in individuals:
                    self.wfile.write(bytes(f"<table border: 1px>", "utf-8"))
                    for key in individual.keys():
                        if key.endswith('_rosstr') and isinstance(individual[key], list):
                            self.wfile.write(bytes(f"<tr><td>{key} length</td><td>{len(individual[key])}</td></tr>", "utf-8"))
                        self.wfile.write(bytes(f"<tr><td>{key}</td><td>{individual[key]}</td></tr>", "utf-8"))
                    self.wfile.write(bytes(f"</table>", "utf-8"))

        # end HTML
        self.wfile.write(bytes("</body></html>", "utf-8"))

    def log_message(self, format, *args):
        return


class SimpleKbWebViewer(PluginBase):

    def on_init_callback(self, plugin_name: str):

        self._kb_server.declare_parameter(f'{plugin_name}.host', 'localhost')
        self._kb_server.declare_parameter(f'{plugin_name}.port', 8080)

        host = self._kb_server.get_parameter(
                f'{plugin_name}.host').get_parameter_value().string_value
        port = self._kb_server.get_parameter(
                f'{plugin_name}.port').get_parameter_value().integer_value

        handler = partial(SimpleWebServer, self._kb_server)
        self.__webServer = HTTPServer((host, port), handler)
        self._kb_server.get_logger().info("SimpleKbWebViewer - Server started http://%s:%s" %
                                          (host, port))

        threading.Thread(target=self.__worker, daemon=True).start()

    def __worker(self):
        self.__webServer.serve_forever()
