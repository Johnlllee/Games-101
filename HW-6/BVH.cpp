#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    if(splitMethod == BVHAccel::SplitMethod::NAIVE) {
        root = recursiveBuild(primitives);
    } else if (splitMethod == BVHAccel::SplitMethod::SAH) {
        std::cout << "Using SAH Method." << std::endl;
        root = recursiveSAHBuild(primitives);
    }


    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveSAHBuild(std::vector<Object*> objects) {
    BVHBuildNode* node = new BVHBuildNode();

    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i) {
        bounds = Union(bounds, objects[i]->getBounds());
    }
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        // return node;
    } else if (objects.size() == 2) {
        node->left = recursiveSAHBuild(std::vector{objects[0]});
        node->right = recursiveSAHBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        // return node;
    } else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });

            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        // SAH part
        auto beginning = objects.begin();
        auto ending = objects.end();
        auto anchor = objects.begin();
        auto min_cost_pos = objects.begin();
        double min_cost = std::numeric_limits<double>::infinity();
        int bin = 8;
        if(dim == 0) {
            float min_x = objects[0]->getBounds().Centroid().x;
            float max_x = objects[objects.size() - 1]->getBounds().Centroid().x;
            float bin_x = (max_x - min_x) / bin;
            for(int i = 0; i < bin; i++) {
                float axis = min_x + bin_x * i;
                bool move = false;
                while(((*anchor)->getBounds().Centroid().x - axis) <= 0) {
                    anchor++;
                    if(anchor == ending) {
                        anchor--;
                        break;
                    }
                    move = true;
                }
                auto leftshapes = std::vector<Object*>(beginning, anchor);
                auto rightshapes = std::vector<Object*>(anchor, ending);
                Bounds3 leftbounds;
                for (int i = 0; i < leftshapes.size(); ++i)
                    leftbounds = Union(leftbounds, leftshapes[i]->getBounds());
                Bounds3 rightbounds;
                for (int i = 0; i < rightshapes.size(); ++i)
                    rightbounds = Union(rightbounds, rightshapes[i]->getBounds());
                double rsa = rightbounds.SurfaceArea();
                double lsa = leftbounds.SurfaceArea();
                double allsa = bounds.SurfaceArea();
                int lsize = leftshapes.size();
                int rsize = rightshapes.size();
                double cost = (lsize * lsa + rsize * rsa) / allsa + 0.125;
                if(cost < min_cost) {
                    min_cost = cost;
                    min_cost_pos = anchor;
                }
                if(anchor + 1 == ending) {
                    break;
                }
                if(move) {
                    anchor--;
                }
            }
        } else if(dim == 1) {
            float min_y = objects[0]->getBounds().Centroid().y;
            float max_y = objects[objects.size() - 1]->getBounds().Centroid().y;
            float bin_y = (max_y - min_y) / bin;
            for(int i = 0; i < bin; i++) {
                float axis = min_y + bin_y * i;
                bool move = false;
                while(((*anchor)->getBounds().Centroid().y - axis) <= 0) {
                    anchor++;
                    if(anchor == ending) {
                        anchor--;
                        break;
                    }
                    move = true;
                }
                auto leftshapes = std::vector<Object*>(beginning, anchor);
                auto rightshapes = std::vector<Object*>(anchor, ending);
                Bounds3 leftbounds;
                for (int i = 0; i < leftshapes.size(); ++i)
                    leftbounds = Union(leftbounds, leftshapes[i]->getBounds());
                Bounds3 rightbounds;
                for (int i = 0; i < rightshapes.size(); ++i)
                    rightbounds = Union(rightbounds, rightshapes[i]->getBounds());
                double rsa = rightbounds.SurfaceArea();
                double lsa = leftbounds.SurfaceArea();
                double allsa = bounds.SurfaceArea();
                int lsize = leftshapes.size();
                int rsize = rightshapes.size();
                double cost = (lsize * lsa + rsize * rsa) / allsa + 0.125;
                if(cost < min_cost) {
                    min_cost = cost;
                    min_cost_pos = anchor;
                }
                if(anchor + 1 == ending) {
                    break;
                }
                if(move) {
                    anchor--;
                }
            }
        } else if(dim == 2) {
            float min_z = objects[0]->getBounds().Centroid().z;
            float max_z = objects[objects.size() - 1]->getBounds().Centroid().z;
            float bin_z = (max_z - min_z) / bin;
            for(int i = 0; i < bin; i++) {
                float axis = min_z + bin_z * i;
                bool move = false;
                while(((*anchor)->getBounds().Centroid().z - axis) <= 0) {
                    anchor++;
                    if(anchor == ending) {
                        anchor--;
                        break;
                    }
                    move = true;
                }
                auto leftshapes = std::vector<Object*>(beginning, anchor);
                auto rightshapes = std::vector<Object*>(anchor, ending);
                Bounds3 leftbounds;
                for (int i = 0; i < leftshapes.size(); ++i)
                    leftbounds = Union(leftbounds, leftshapes[i]->getBounds());
                Bounds3 rightbounds;
                for (int i = 0; i < rightshapes.size(); ++i)
                    rightbounds = Union(rightbounds, rightshapes[i]->getBounds());
                double rsa = rightbounds.SurfaceArea();
                double lsa = leftbounds.SurfaceArea();
                double allsa = bounds.SurfaceArea();
                int lsize = leftshapes.size();
                int rsize = rightshapes.size();
                double cost = (lsize * lsa + rsize * rsa) / allsa + 0.125;
                if(cost < min_cost) {
                    min_cost = cost;
                    min_cost_pos = anchor;
                }
                if(anchor + 1 == ending) {
                    break;
                }
                if(move) {
                    anchor--;
                }
            }
        }
        auto leftshapes = std::vector<Object*>(beginning, min_cost_pos);
        auto rightshapes = std::vector<Object*>(min_cost_pos, ending);
        assert(objects.size() == (leftshapes.size() + rightshapes.size()));
        node->left = recursiveSAHBuild(leftshapes);
        node->right = recursiveSAHBuild(rightshapes);
        node->bounds = Union(node->left->bounds, node->right->bounds);
    }
    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    std::array<int, 3> dirIsNeg = {0, 0, 0};
    if (node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg)) {
        if(node->left == nullptr && node->right == nullptr) {
            return node->object->getIntersection(ray);
        } else {
            Intersection left, right;
            if(node->left != nullptr) {
                left = getIntersection(node->left, ray);
            }
            if(node->right != nullptr) {
                right = getIntersection(node->right, ray);
            }
            if(left.happened && right.happened) {
                if(left.distance < right.distance) {
                    return left;
                } else {
                    return right;
                }
            } else if(left.happened) {
                return left;
            } else if(right.happened) {
                return right;
            } else {
                Intersection isect;
                return isect;
            }
        }
    } else {
        Intersection isect;
        return isect;
    }
}