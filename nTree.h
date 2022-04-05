/*
 *  nTree.h
 *  structureTest
 *
 *  Created by Brian Kirkpatrick on 3/10/09.
 *  Copyright 2009 Tythos Creatives, Brian Kirkpatrick. All rights reserved.
 *
 *  nTree defines a templated n-degree tree node that contains a pointer to
 *  it's item and parent node, as well as a smartList (modified linked list)
 *  containing the addresses of its children. For ease of access and
 *  calculation, the level of the tree in which this node resides is also
 *  stored. An nTree should be designed around the use of nodes in
 *  coordinate frame hierarchies, which should be supported in an extended
 *  version of this class.
 *
 *  Note: node ids shold be unique within a given tree, meaning that they
 *  must be tracked and updated when trees are merged.
 *
 *  A key principle behind this class is that the end user should never
 *  use or manipulate a tree node other than the head. This should promote
 *  security and consistant, clear use of trees in code.
 */

#ifndef N_TREE_H
#define N_TREE_H

#include "smartList.h"

template <class T> class nTree {
private:
	T * pItem;
	nTree * parent;
	smartList<nTree> children;
	int level;
	int nodeId;
protected:
	// Node-level functions, called on a specific tree node
	nTree<T> * getNode(int requestedId);
	nTree<T> * headNode();
	int numChildren();
	int getLevel();
	void printNode();	
public:
	// Constructors
	nTree();
	nTree(T * newItem);
	~nTree();
	
	// Tree-level functions, must always be called on head
	T * getItem(int nodeId);
	bool isNode(int testId);
	int nextAvailableId();
	int getSize();
	int addChild(T * newItem, int parentId);
	int addChild(T * newItem);
	void removeNode(int requestedId);
	void merge(nTree<T> * subTree, int parentId);
	void printNode(int nodeId);
	void printTree();
	void reset();
};

#endif