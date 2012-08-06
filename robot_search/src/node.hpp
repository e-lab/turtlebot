#include<list>
#include<map>
#include "point.hpp"

const float HASH_SIZE = 6;
const float CHECK_SIZE = 0.75;//0.75;

typedef struct{
	int x_key, y_key;
}Index;

struct compare{
	bool operator()(Index a, Index b){
		return (a.x_key<b.x_key);
	}
};


class Node{
	private:
		Point point;
		Node* parent_ptr;
		std::list<Node*> children_ptr;
		bool explore_status;
		static std::multimap<Index, Node, compare> node_list;
	public:
		static void add_node(Node input_node){
			int x_key = input_node.point.x() / HASH_SIZE;
			int y_key = input_node.point.y() / HASH_SIZE;
			Index key;
			key.x_key = x_key;
			key.y_key = y_key;
			std::pair<Index,Node> node_data(key,input_node);
			node_list.insert(node_data);
		}

		static std::multimap<Index, Node, compare> getList(){
			return node_list;
		}

		static bool checkNearby(double x_input, double y_input){
			int x_key = x_input / HASH_SIZE;
			int y_key = y_input / HASH_SIZE;
			int x_flag = 0, y_flag = 0; // this flag checks whether a check on adjacent hash elements is needed
			if((int)((x_input - CHECK_SIZE)/HASH_SIZE) != x_key)
				x_flag = -1;
			if((int)((x_input + CHECK_SIZE)/HASH_SIZE) != x_key)
				x_flag = 1;
			if((int)((y_input - CHECK_SIZE)/HASH_SIZE) != y_key)
				y_flag = -1;
			if((int)((y_input + CHECK_SIZE)/HASH_SIZE) != y_key)
				y_flag = 1;
			Index index;
			index.x_key = x_key;
			index.y_key = y_key;
			std::pair<std::multimap<Index,Node,compare>::iterator,std::multimap<Index,Node,compare>::iterator> it_pair;
			std::multimap<Index,Node,compare>::iterator it;
			it_pair = node_list.equal_range(index);
			for(it=it_pair.first; it!=it_pair.second; it++){
				if((it->second.point.x() >= x_input - CHECK_SIZE && it->second.point.x() <= x_input + CHECK_SIZE)
					&&(it->second.point.y() >= y_input - CHECK_SIZE && it->second.point.y() <= y_input + CHECK_SIZE))
					return true;
			}
			// this if statement checks on adjacent hash elements
			if(x_flag!=0 || y_flag!=0){
				index.x_key = x_key + x_flag;
				index.y_key = y_key + y_flag;
				it_pair = node_list.equal_range(index);
				for(it=it_pair.first; it!=it_pair.second; it++){
					if((it->second.point.x() >= x_input - CHECK_SIZE && it->second.point.x() <= x_input + CHECK_SIZE)
					&&(it->second.point.y() >= y_input - CHECK_SIZE && it->second.point.y() <= y_input + CHECK_SIZE))
						return true;
				}
			}
			return false;
		}

		Node(double x_input, double y_input):point(x_input, y_input){
			parent_ptr = NULL;
			explore_status = false;
		}

		Node* addChild(double x_child, double y_child){
			Node* child = new Node(x_child, y_child);
			child->parent_ptr = this;
			children_ptr.push_back(child);
			return child;
		}

		Node* getParent(){
			return parent_ptr;
		}

		std::list<Node*> getChildren(){
			return children_ptr;
		}

		Point getPoint(){
			return point;
		}

		bool getStatus(){
			return explore_status;
		}

		void explored(bool status){
			explore_status = status;
		}
};
