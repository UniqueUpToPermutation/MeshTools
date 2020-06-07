#include <iostream>
#include <memory>

#include "hfe.h"

int main()
{
	auto geo = std::unique_ptr<hfe::Geometry>(hfe::LoadJson("cube.json"));
	// geo->SaveJson("test.json");

	for (auto face = geo->getFace(0); face.isValid(); face = face.nextById()) {
		std::cout << "face " << face.id() << ": ";
		std::cout << "verts [";
		for (auto vert = face.vertices(); vert.isValid(); vert.next()) {
			std::cout << vert().id() << " ";
		}
		std::cout << "], edges [";
		for (auto edge = face.edges(); edge.isValid(); edge.next()) {
			std::cout << edge().id() << " ";
		}
		std::cout << "]" << std::endl;
	}

	std::cout << std::endl;

	for (auto edge = geo->getEdge(0); edge.isValid(); edge = edge.nextById()) {
		std::cout << "edge " << edge.id() << ": ";
		std::cout << "tail = " << edge.tail().id() << " head = " << edge.head().id() << " opposite = " << edge.opposite().id();
		std::cout << std::endl;
	}

	std::cout << std::endl;

	for (auto vert = geo->getVertex(0); vert.isValid(); vert = vert.nextById()) {
		std::cout << "vertex " << vert.id() << ": " << std::endl;
		std::cout << "adj. verts: [";
		for (auto adjVert = vert.neighbors(); adjVert.isValid(); adjVert.next())
			std::cout << adjVert().id() << " ";
		std::cout << "]" << std::endl << "adj. faces: [";
		for (auto face = vert.faces(); face.isValid(); face.next())
			std::cout << face().id() << " ";
		std::cout << "]" << std::endl << "outgoing edges: [";
		for (auto edge = vert.outgoing(); edge.isValid(); edge.next())
			std::cout << edge().id() << " ";
		std::cout << "]" << std::endl;
	}
}
