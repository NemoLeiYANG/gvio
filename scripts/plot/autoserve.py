#!/usr/bin/env python
from livereload import Server, shell
server = Server()
server.watch('./*.html')
server.serve(root='.', port=35729)
