#!/usr/bin/env python
from livereload import Server, shell
server = Server()
server.watch('./*.py', shell('python3 plot_fig.py', cwd="."))
server.serve(root='.')
