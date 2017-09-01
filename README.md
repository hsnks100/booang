# booang

boost:graph for beginners

## linux ubuntu install



```
sudo apt install libgraphviz_dev 
git clone https://github.com/hsnks100/booang.git
cd booang
make
```

## windows install
![image](https://user-images.githubusercontent.com/3623889/29987887-52648424-8fa5-11e7-95de-05fcbfdc9788.png)

![image](https://user-images.githubusercontent.com/3623889/29987897-67a02078-8fa5-11e7-9f9d-120b7f1d138c.png)

![image](https://user-images.githubusercontent.com/3623889/29987910-7a5e3196-8fa5-11e7-81cc-b6b716890817.png)

![image](https://user-images.githubusercontent.com/3623889/29987919-857603c4-8fa5-11e7-859c-19d7dc767350.png)

```
#define BOOPATH "your\\booang\\path"
#include <booang.hpp>

struct VertexProperty {
	int Id;
	// int toString;
	std::string toString() {
		return std::to_string(Id);
	}
	// friend ostream& operator<<(ostream& oss, const VertexProperty& vp) {
	//     return oss << vp.Id;
	// }
};
int main(int, char*[])
{	
	WeightedGraph G2;

	G2.resize(5);
	G2.addEdge(0, 1, 1);
	G2.addEdge(1, 2, 1);
	G2.addEdge(1, 3, 1);
	G2.addEdge(2, 4, 1);
	G2.addEdge(3, 4, 1);
	G2.printGraphViz("2.png");

}
```

![grim18](https://user-images.githubusercontent.com/3623889/29940642-d338963e-8eca-11e7-80f5-f96d8f4f977a.png)
