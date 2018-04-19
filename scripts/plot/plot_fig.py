class Line:
    def __init__(self, start, end, **kwargs):
        self.start = start
        self.end = end
        self.color = kwargs.get("color", (0, 0, 0))
        self.width = kwargs.get("width", 2)

    def draw(self):
        style = "stroke:rgb%s;" % str(self.color)
        style += "stroke-width:%d" % self.width

        svg = '<line '
        svg += 'x1="%d" ' % self.start[0]
        svg += 'y1="%d" ' % self.start[1]
        svg += 'x2="%d" ' % self.end[0]
        svg += 'y2="%d" ' % self.end[1]
        svg += 'style="%s"' % style
        svg += '/>'
        return svg


class Cord2DAxis:
    def __init__(self, start, end, **kwargs):
        self.start = start
        self.end = end
        self.color = kwargs.get("color", (0, 0, 0))
        self.width = kwargs.get("width", 2)

    def draw(self):
        svg = ""
        svg += Line((40, 10), (40, 110)).draw() + "\n")
        svg += Line((30, 100), (200, 100)).draw() + "\n")
        return svg


drawing = open("index.html", "w")
drawing.write('<html>\n')
drawing.write('<script src="//localhost:35729/livereload.js"></script>\n')
drawing.write('<svg height="500" width="500">\n')
drawing.write(Line((40, 10), (40, 110)).draw() + "\n")
drawing.write(Line((30, 100), (200, 100)).draw() + "\n")
drawing.write('</svg>\n')
drawing.write('</html>\n')
drawing.close()
