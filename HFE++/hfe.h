#ifndef H_HFE
#define H_HFE

#include <vector>
#include <functional>

namespace hfe
{
	struct Vector3d {
		double x;
		double y;
		double z;
	};

	struct Vector2d {
		double x;
		double y;
	};

	struct RawVertex {
		int edge;

		inline RawVertex(const int edge) : edge(edge) {}
	};

	struct RawEdge {
		int face;
		int head;
		int next;
		int opposite;

		inline RawEdge(const int face, const int head, const int next, const int opposite) : 
			face(face), head(head), next(next), opposite(opposite) {}
	};

	struct RawFace {
		int edge;

		inline RawFace(const int edge) : edge(edge) {}
	};

	struct BoundingBox {
		Vector3d upper;
		Vector3d lower;
	};

	class Vertex;
	class Edge;
	class Face;
	class EdgeIterator;
	class VertexIterator;
	class FaceIterator;
	class Geometry;

	class Vertex {
	private:
		Geometry* geo_;
		int id_;

	public:
		explicit inline Vertex(Geometry* geo, int id) : geo_(geo), id_(id) {}

		inline int id() const { return id_; }
		
		inline Vector3d* position();
		inline Vector2d* uv();
		inline Vector3d* normal();
		inline Vector3d* tangent();
		inline RawVertex* raw();
		inline void setEdge(const Edge& e);
		inline Edge edge();
		inline EdgeIterator outgoing();
		inline EdgeIterator incoming();
		inline VertexIterator neighbors();
		inline FaceIterator faces();
		inline Vertex nextById();
		inline bool isValid();
		
		friend class Edge;
		friend class Face;
		friend class VertexIterator;
		friend class EdgeIterator;
		friend class FaceIterator;
		friend class Geometry;
	};

	class Edge {
	private:
		Geometry* geo_;
		int id_;

	public:
		explicit inline Edge(Geometry* geo, int id) : geo_(geo), id_(id) {}

		inline int id() const { return id_; }

		inline RawEdge* raw();
		inline void setOpposite(const Edge& e);
		inline void setHead(const Vertex& v);
		inline void setNext(const Edge& e);
		inline void setFace(const Face& f);
		inline Edge opposite();
		inline Vertex head();
		inline Vertex tail();
		inline Face face();
		inline Edge next();
		inline Edge nextById();
		inline bool isValid();

		friend class Vertex;
		friend class Face;
		friend class VertexIterator;
		friend class EdgeIterator;
		friend class FaceIterator;
		friend class Geometry;
	};

	class Face {
	private:
		Geometry* geo_;
		int id_;

	public:
		explicit inline Face(Geometry* geo, int id) : geo_(geo), id_(id) {}

		inline int id() const { return id_; }

		inline RawFace* raw();
		inline void setEdge(const Edge& e);
		inline Edge edge();
		inline EdgeIterator edges();
		inline FaceIterator adjacent();
		inline VertexIterator vertices();
		inline Face nextById();
		inline bool isValid();

		friend class Vertex;
		friend class Edge;
		friend class VertexIterator;
		friend class EdgeIterator;
		friend class FaceIterator;
		friend class Geometry;
	};

	class VertexIterator {
	private:
		Edge currentEdge;
		Edge startEdge;

		void nextAdjacent() {
			currentEdge = currentEdge.opposite().next();
			if (currentEdge.id_ == startEdge.id_)
				currentEdge.id_ = -1;
		}

		void nextOnFace() {
			currentEdge = currentEdge.next();
			if (currentEdge.id_ == startEdge.id_)
				currentEdge.id_ = -1;
		}

		void(VertexIterator::*nextIt)();

	public:

		inline explicit VertexIterator(Edge startEdge, void(VertexIterator::*next)()) : 
			currentEdge(startEdge), startEdge(startEdge), nextIt(next) {}
		
		inline void next() { (this->*nextIt)(); }

		inline bool done() const { return currentEdge.id_ < 0; }
		inline bool isValid() const { return !done(); }
		inline Vertex operator()() { return currentEdge.head(); }

		friend class Vertex;
		friend class Edge;
		friend class Face;
	};

	class EdgeIterator {
	private:
		Edge currentEdge;
		Edge startEdge;
		void(EdgeIterator::*nextIt)();

		void nextOnFace() {
			currentEdge = currentEdge.next();
			if (currentEdge.id_ == startEdge.id_)
				currentEdge.id_ = -1;
		}

		void nextOutgoing() {
			currentEdge = currentEdge.opposite().next();
			if (currentEdge.id_ == startEdge.id_)
				currentEdge.id_ = -1;
		}

		void nextIngoing() {
			currentEdge = currentEdge.next().opposite();
			if (currentEdge.id_ == startEdge.id_)
				currentEdge.id_ = -1;
		}

	public:
		inline explicit EdgeIterator(Edge startEdge, void(EdgeIterator::*next)()) :
			currentEdge(startEdge), startEdge(startEdge), nextIt(next) {}

		inline void next() { (this->*nextIt)(); }

		inline Edge operator()() { return currentEdge; }
		inline bool done() const { return currentEdge.id_ < 0; }
		inline bool isValid() const { return !done(); }

		friend class Vertex;
		friend class Edge;
		friend class Face;
	};

	class FaceIterator {
	private:
		Edge currentEdge;
		Edge startEdge;
		void(FaceIterator::*nextIt)();

		void faceNextAdjacentFace() {
			currentEdge = currentEdge.next();
			if (currentEdge.id_ == startEdge.id_)
				currentEdge.id_ = -1;
		}

		void vertexNextAdjacentFace() {
			currentEdge = currentEdge.opposite().next();
			if (currentEdge.id_ == startEdge.id_)
				currentEdge.id_ = -1;
		}

	public:
		inline explicit FaceIterator(Edge startEdge, void(FaceIterator::*next)()) :
			currentEdge(startEdge), startEdge(startEdge), nextIt(next) {}
		
		inline void next() { (this->*nextIt)(); }

		inline bool done() const { return currentEdge.id_ < 0; }
		inline bool isValid() const { return !done(); }
		inline Face operator()() { return currentEdge.opposite().face(); }

		friend class Vertex;
		friend class Edge;
		friend class Face;
	};

	class Geometry {
	private:
		std::vector<Vector3d> vertexPositions;
		std::vector<Vector2d> vertexUVs;
		std::vector<Vector3d> vertexNormals;
		std::vector<Vector3d> vertexTangents;
		std::vector<RawVertex> vertices;
		std::vector<RawEdge> edges;
		std::vector<RawFace> faces;

		BoundingBox aabb;

	public:
		inline BoundingBox getBoundingBox() const {
			return aabb;
		}
		inline bool hasPositions() const {
			return vertexPositions.size() > 0;
		}
		inline bool hasUVs() const {
			return vertexUVs.size() > 0;
		}
		inline bool hasNormals() const {
			return vertexNormals.size() > 0;
		}
		inline bool hasTangents() const {
			return vertexTangents.size() > 0;
		}
		inline Vertex getVertex(const int id) {
			return Vertex(this, id);
		}
		inline Edge getEdge(const int id) {
			return Edge(this, id);
		}
		inline Face getFace(const int id) {
			return Face(this, id);
		}
		inline size_t vertexCount() const {
			return vertices.size();
		}
		inline size_t edgeCount() const {
			return edges.size();
		}
		inline size_t faceCount() const {
			return faces.size();
		}
		void updateBoundingBox();

		void Save(const std::string& path);
		void SaveJson(const std::string& path);

		friend class Vertex;
		friend class Edge;
		friend class Face;

		friend Geometry* Load(const std::string& path);
		friend Geometry* LoadJson(const std::string& filename);
	};

	Geometry* Load(const std::string& path);
	Geometry* LoadJson(const std::string& path);

	inline Vector3d* Vertex::position() {
		return &geo_->vertexPositions[id_];
	}
	inline Vector2d* Vertex::uv() {
		return &geo_->vertexUVs[id_];
	}
	inline Vector3d* Vertex::normal() {
		return &geo_->vertexNormals[id_];
	}
	inline Vector3d* Vertex::tangent() {
		return &geo_->vertexTangents[id_];
	}
	inline RawVertex* Vertex::raw() {
		return &geo_->vertices[id_];
	}
	inline void Vertex::setEdge(const Edge& e) {
		raw()->edge = e.id_;
	}
	inline Edge Vertex::edge() {
		return Edge(geo_, raw()->edge);
	}
	inline EdgeIterator Vertex::outgoing() {
		return EdgeIterator(edge(), &EdgeIterator::nextOutgoing);
	}
	inline EdgeIterator Vertex::incoming() {
		return EdgeIterator(edge().opposite(), &EdgeIterator::nextIngoing);
	}
	inline VertexIterator Vertex::neighbors() {
		return VertexIterator(edge(), &VertexIterator::nextAdjacent);
	}
	inline FaceIterator Vertex::faces() {
		return FaceIterator(edge(), &FaceIterator::vertexNextAdjacentFace);
	}
	inline Vertex Vertex::nextById() {
		return Vertex(geo_, id_ + 1);
	}
	inline bool Vertex::isValid() {
		return id_ >= 0 && id_ < (int)geo_->vertices.size();
	}

	inline RawEdge* Edge::raw() {
		return &geo_->edges[id_];
	}
	inline void Edge::setOpposite(const Edge& e) {
		raw()->opposite = e.id_;
	}
	inline void Edge::setHead(const Vertex& v) {
		raw()->head = v.id_;
	}
	inline void Edge::setNext(const Edge& e) {
		raw()->next = e.id_;
	}
	inline void Edge::setFace(const Face& f) {
		raw()->face = f.id_;
	}
	inline Edge Edge::opposite() {
		return Edge(geo_, raw()->opposite);
	}
	inline Vertex Edge::head() {
		return Vertex(geo_, raw()->head);
	}
	inline Vertex Edge::tail() {
		return Vertex(geo_, opposite().head().id_);
	}
	inline Face Edge::face() {
		return Face(geo_, raw()->face);
	}
	inline Edge Edge::next() {
		return Edge(geo_, raw()->next);
	}
	inline Edge Edge::nextById() {
		return Edge(geo_, id_ + 1);
	}
	inline bool Edge::isValid() {
		return id_ >= 0 && id_ < (int)geo_->edges.size();
	}

	inline RawFace* Face::raw() {
		return &geo_->faces[id_];
	}
	inline void Face::setEdge(const Edge& e) {
		raw()->edge = e.id_;
	}
	inline Edge Face::edge() {
		return Edge(geo_, raw()->edge);
	}
	inline EdgeIterator Face::edges() {
		return EdgeIterator(edge(), &EdgeIterator::nextOnFace);
	}
	inline FaceIterator Face::adjacent() {
		return FaceIterator(edge(), &FaceIterator::faceNextAdjacentFace);
	}
	inline VertexIterator Face::vertices() {
		return VertexIterator(edge(), &VertexIterator::nextOnFace);
	}
	inline Face Face::nextById() {
		return Face(geo_, id_ + 1);
	}
	inline bool Face::isValid() {
		return id_ >= 0 && id_ < (int)geo_->faces.size();
	}
}

#endif