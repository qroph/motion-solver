/*
Copyright (c) 2014 Mika Rantanen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef __DISJOINTSETS_H__
#define __DISJOINTSETS_H__

class Roadmap;
class RoadmapNode;

struct DisjointSetsNode
{
    friend class DisjointSets;
    friend class RoadmapNode;

private:
    DisjointSetsNode():
        disjointSetsParent(this),
        disjointSetsRank(0)
    {
    }

    DisjointSetsNode* disjointSetsParent;
    int disjointSetsRank;
};


class DisjointSets
{
    friend class Roadmap;

private:
    DisjointSets():
        m_setCount(0)
    {
    }

    int getSetCount() const
    {
        return m_setCount;
    }

    void makeSet(DisjointSetsNode& element)
    {
        element.disjointSetsParent = &element;
        element.disjointSetsRank = 0;
        ++m_setCount;
    }

    void uniteSets(DisjointSetsNode& element1,
                   DisjointSetsNode& element2)
    {
        DisjointSetsNode* set1 = findSet(&element1);
        DisjointSetsNode* set2 = findSet(&element2);

        if (set1 != set2)
        {
            if (set1->disjointSetsRank > set2->disjointSetsRank)
            {
                set2->disjointSetsParent = set1;
            }
            else
            {
                set1->disjointSetsParent = set2;

                if (set1->disjointSetsRank == set2->disjointSetsRank)
                {
                    ++set2->disjointSetsRank;
                }
            }

            --m_setCount;
        }
    }

    bool sameComponent(DisjointSetsNode& element1,
                       DisjointSetsNode& element2)
    {
        return findSet(&element1) == findSet(&element2);
    }

    DisjointSetsNode* findSet(DisjointSetsNode* element)
    {
        if (element != element->disjointSetsParent)
        {
            element->disjointSetsParent = findSet(element->disjointSetsParent);
        }

        return element->disjointSetsParent;
    }

    int m_setCount;
};

#endif
