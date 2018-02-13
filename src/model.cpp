/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author Ioan Sucan */
#include <srdfdom_advr/model.h>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <set>
#include <limits>
#include <string>

std::string trim(const std::string &str) {
    size_t first = str.find_first_not_of(' ');
    if (std::string::npos == first) {
        return str;
    }
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, (last - first + 1));
}

void srdf_advr::Model::loadVirtualJoints(const urdf::ModelInterface &urdf_model, tinyxml2::XMLElement *robot_xml) {
    for (tinyxml2::XMLElement *vj_xml = robot_xml->FirstChildElement("virtual_joint"); vj_xml; vj_xml = vj_xml->NextSiblingElement("virtual_joint")) {
        const char *jname = vj_xml->Attribute("name");
        const char *child = vj_xml->Attribute("child_link");
        const char *parent = vj_xml->Attribute("parent_frame");
        const char *type = vj_xml->Attribute("type");
        if (!jname) {
            std::cerr << "Name of virtual joint is not specified" << std::endl;
            //logError("Name of virtual joint is not specified");
            continue;
        }
        if (!child) {
            std::cerr << "Child link of virtual joint is not specified" << std::endl;
            //logError("Child link of virtual joint is not specified");
            continue;
        }
        if (!urdf_model.getLink(trim(std::string(child)))) {
            std::cerr << "Virtual joint does not attach to a link on the robot" << std::endl;
            //logError("Virtual joint does not attach to a link on the robot (link '%s' is not known)", child);
            continue;
        }
        if (!parent) {
            std::cerr << "Parent frame of virtual joint is not specified" << std::endl;
            //logError("Parent frame of virtual joint is not specified");
            continue;
        }
        if (!type) {
            std::cerr << "Type of virtual joint is not specified" << std::endl;
            //logError("Type of virtual joint is not specified");
            continue;
        }
        VirtualJoint vj;
        vj.type_ = std::string(type); trim(vj.type_);
        std::transform(vj.type_.begin(), vj.type_.end(), vj.type_.begin(), ::tolower);
        if (vj.type_ != "planar" && vj.type_ != "floating" && vj.type_ != "fixed") {
            std::cerr << "Assuming 'fixed' joint" << std::endl;
            //logError("Unknown type of joint: '%s'. Assuming 'fixed' instead. Other known types are 'planar' and 'floating'.", type);
            vj.type_ = "fixed";
        }
        vj.name_ = std::string(jname); trim(vj.name_);
        vj.child_link_ = std::string(child); trim(vj.child_link_);
        vj.parent_frame_ = std::string(parent); trim(vj.parent_frame_);
        virtual_joints_.push_back(vj);
    }
}

void srdf_advr::Model::loadGroups(const urdf::ModelInterface &urdf_model, tinyxml2::XMLElement *robot_xml) {
    for (tinyxml2::XMLElement *group_xml = robot_xml->FirstChildElement("group"); group_xml; group_xml = group_xml->NextSiblingElement("group")) {
        const char *gname = group_xml->Attribute("name");
        if (!gname) {
            std::cerr << "Group name not specified" << std::endl;
            //logError("Group name not specified");
            continue;
        }
        Group g;
        g.name_ = std::string(gname); trim(g.name_);

        // get the links in the groups
        for (tinyxml2::XMLElement *link_xml = group_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link")) {
            const char *lname = link_xml->Attribute("name");
            if (!lname) {
                std::cerr << "Link name not specified" << std::endl;
                //logError("Link name not specified");
                continue;
            }
            std::string lname_str = trim(std::string(lname));
            if (!urdf_model.getLink(lname_str)) {
                std::cerr << "Link of group is not known" << std::endl;
                //logError("Link '%s' declared as part of group '%s' is not known to the URDF", lname, gname);
                continue;
            }
            g.links_.push_back(lname_str);
        }

        // get the joints in the groups
        for (tinyxml2::XMLElement *joint_xml = group_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint")) {
            const char *jname = joint_xml->Attribute("name");
            if (!jname) {
                std::cerr << "Joint name not specified" << std::endl;
                //logError("Joint name not specified");
                continue;
            }
            std::string jname_str = trim(std::string(jname));
            if (!urdf_model.getJoint(jname_str)) {
                bool missing = true;
                for (std::size_t k = 0; k < virtual_joints_.size(); ++k)
                    if (virtual_joints_[k].name_ == jname_str) {
                        missing = false;
                        break;
                    }
                if (missing) {
                    std::cerr << "Joint as part of group not known" << std::endl;
                    //logError("Joint '%s' declared as part of group '%s' is not known to the URDF", jname, gname);
                    continue;
                }
            }
            g.joints_.push_back(jname_str);
        }

        // get the chains in the groups
        for (tinyxml2::XMLElement *chain_xml = group_xml->FirstChildElement("chain"); chain_xml; chain_xml = chain_xml->NextSiblingElement("chain")) {
            const char *base = chain_xml->Attribute("base_link");
            const char *tip = chain_xml->Attribute("tip_link");
            if (!base) {
                std::cerr << "Base link name not specified for chain" << std::endl;
                //logError("Base link name not specified for chain");
                continue;
            }
            if (!tip) {
                std::cerr << "Tip link name not specified for chain" << std::endl;
                //logError("Tip link name not specified for chain");
                continue;
            }
            std::string base_str = trim(std::string(base));
            std::string tip_str = trim(std::string(tip));
            if (!urdf_model.getLink(base_str)) {
                std::cerr << "Link declared as part of chain not known" << std::endl;
                //logError("Link '%s' declared as part of a chain in group '%s' is not known to the URDF", base, gname);
                continue;
            }
            if (!urdf_model.getLink(tip_str)) {
                std::cerr << "Link declared as part of chain not known" << std::endl;
                //logError("Link '%s' declared as part of a chain in group '%s' is not known to the URDF", tip, gname);
                continue;
            }
            bool found = false;
            std::shared_ptr<const urdf::Link> l = urdf_model.getLink(tip_str);
            std::set<std::string> seen;
            while (!found && l) {
                seen.insert(l->name);
                if (l->name == base_str)
                    found = true;
                else
                    l = l->getParent();
            }
            if (!found) {
                l = urdf_model.getLink(base_str);
                while (!found && l) {
                    if (seen.find(l->name) != seen.end())
                        found = true;
                    else
                        l = l->getParent();
                }
            }
            if (found)
                g.chains_.push_back(std::make_pair(base_str, tip_str));
            else
                std::cerr << "Links do not form a chain" << std::endl;
            //logError("Links '%s' and '%s' do not form a chain. Not included in group '%s'", base, tip, gname);
        }

        // get the subgroups in the groups
        for (tinyxml2::XMLElement *subg_xml = group_xml->FirstChildElement("group"); subg_xml; subg_xml = subg_xml->NextSiblingElement("group")) {
            const char *sub = subg_xml->Attribute("name");
            if (!sub) {
                std::cerr << "Group name not specified when included as subgroup" << std::endl;
                //logError("Group name not specified when included as subgroup");
                continue;
            }
            g.subgroups_.push_back(trim(std::string(sub)));
        }
        if (g.links_.empty() && g.joints_.empty() && g.chains_.empty() && g.subgroups_.empty())
            std::cerr << "Group is empty" << std::endl;
        // logWarn("Group '%s' is empty.", gname);
        groups_.push_back(g);
        //std::cout<<groups_.size()<<"GROUP!!\n";
    }

    // check the subgroups
    std::set<std::string> known_groups;
    bool update = true;
    while (update) {
        update = false;
        for (std::size_t i = 0; i < groups_.size(); ++i) {
            if (known_groups.find(groups_[i].name_) != known_groups.end())
                continue;
            if (groups_[i].subgroups_.empty()) {
                known_groups.insert(groups_[i].name_);
                update = true;
            } else {
                bool ok = true;
                for (std::size_t j = 0; ok && j < groups_[i].subgroups_.size(); ++j)
                    if (known_groups.find(groups_[i].subgroups_[j]) == known_groups.end())
                        ok = false;
                if (ok) {
                    known_groups.insert(groups_[i].name_);
                    update = true;
                }
            }
        }
    }

    // if there are erroneous groups, keep only the valid ones
    if (known_groups.size() != groups_.size()) {
        std::vector<Group> correct;
        for (std::size_t i = 0; i < groups_.size(); ++i)
            if (known_groups.find(groups_[i].name_) != known_groups.end())
                correct.push_back(groups_[i]);
            else
                std::cerr << "Group has unsatisfied subgroups" << std::endl;
        //logError("Group '%s' has unsatisfied subgroups", groups_[i].name_.c_str());
        groups_.swap(correct);
    }
}

void srdf_advr::Model::loadGroupStates(const urdf::ModelInterface &urdf_model, tinyxml2::XMLElement *robot_xml) {
    for (tinyxml2::XMLElement *gstate_xml = robot_xml->FirstChildElement("group_state"); gstate_xml; gstate_xml = gstate_xml->NextSiblingElement("group_state")) {
        const char *sname = gstate_xml->Attribute("name");
        const char *gname = gstate_xml->Attribute("group");
        if (!sname) {
            std::cerr << "Name of group state is not specified" << std::endl;
            //logError("Name of group state is not specified");
            continue;
        }
        if (!gname) {
            std::cerr << "Name of group for state '%s' is not specified" << std::endl;
            //logError("Name of group for state '%s' is not specified", sname);
            continue;
        }

        GroupState gs;
        gs.name_ = trim(std::string(sname));
        gs.group_ = trim(std::string(gname));

        bool found = false;
        for (std::size_t k = 0; k < groups_.size(); ++k)
            if (groups_[k].name_ == gs.group_) {
                found = true;
                break;
            }
        if (!found) {
            std::cerr << "Group state specified for group, but that group is not known" << std::endl;
            //logError("Group state '%s' specified for group '%s', but that group is not known", sname, gname);
            continue;
        }

        // get the joint values in the group state
        for (tinyxml2::XMLElement *joint_xml = gstate_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint")) {
            const char *jname = joint_xml->Attribute("name");
            const char *jval = joint_xml->Attribute("value");
            if (!jname) {
                std::cerr << "Joint name not specified in group state" << std::endl;
                //logError("Joint name not specified in group state '%s'", sname);
                continue;
            }
            if (!jval) {
                std::cerr << "Joint name not specified for joint in group state" << std::endl;
                //logError("Joint name not specified for joint '%s' in group state '%s'", jname, sname);
                continue;
            }
            std::string jname_str = trim(std::string(jname));
            if (!urdf_model.getJoint(jname_str)) {
                bool missing = true;
                for (std::size_t k = 0; k < virtual_joints_.size(); ++k)
                    if (virtual_joints_[k].name_ == jname_str) {
                        missing = false;
                        break;
                    }
                if (missing) {
                    std::cerr << "Joint declared as part of group state is not known to the URDF" << std::endl;
                    //logError("Joint '%s' declared as part of group state '%s' is not known to the URDF", jname, sname);
                    continue;
                }
            }
            try{
                std::string jval_str = std::string(jval);
                std::stringstream ss(jval_str);
                while (ss.good() && !ss.eof()) {
                    std::string val; ss >> val >> std::ws;
                    gs.joint_values_[jname_str].push_back(std::stod(val));
                }
            }catch (std::invalid_argument &e) {
                std::cerr << "Unable to parse joint value" << std::endl;
                //logError("Unable to parse joint value '%s'", jval);
            }catch (std::out_of_range &e) {
                std::cerr << "Unable to parse joint value" << std::endl;
                //logError("Unable to parse joint value '%s'", jval);
            }

            if (gs.joint_values_.empty())
                std::cerr << "Unable to parse joint value for joint in group state" << std::endl;
            //logError("Unable to parse joint value ('%s') for joint '%s' in group state '%s'", jval, jname, sname);
        }
        group_states_.push_back(gs);
    }
}

void srdf_advr::Model::loadEndEffectors(const urdf::ModelInterface &urdf_model, tinyxml2::XMLElement *robot_xml) {
    for (tinyxml2::XMLElement *eef_xml = robot_xml->FirstChildElement("end_effector"); eef_xml; eef_xml = eef_xml->NextSiblingElement("end_effector")) {
        const char *ename = eef_xml->Attribute("name");
        const char *gname = eef_xml->Attribute("group");
        const char *parent = eef_xml->Attribute("parent_link");
        const char *parent_group = eef_xml->Attribute("parent_group");
        if (!ename) {
            std::cerr << "Name of end effector is not specified" << std::endl;
            //logError("Name of end effector is not specified");
            continue;
        }
        if (!gname) {
            std::cerr << "Group not specified for end effector" << std::endl;
            //logError("Group not specified for end effector '%s'", ename);
            continue;
        }
        EndEffector e;
        e.name_ = std::string(ename); trim(e.name_);
        e.component_group_ = std::string(gname); trim(e.component_group_);
        bool found = false;
        for (std::size_t k = 0; k < groups_.size(); ++k)
            if (groups_[k].name_ == e.component_group_) {
                found = true;
                break;
            }
        if (!found) {
            std::cerr << "End effector specified for group, but that group is not known" << std::endl;
            //logError("End effector '%s' specified for group '%s', but that group is not known", ename, gname);
            continue;
        }
        if (!parent) {
            std::cerr << "Parent link not specified for end effector" << std::endl;
            //logError("Parent link not specified for end effector '%s'", ename);
            continue;
        }
        e.parent_link_ = std::string(parent); trim(e.parent_link_);
        if (!urdf_model.getLink(e.parent_link_)) {
            std::cerr << "Link specified as parent for end effector is not known to the URDF" << std::endl;
            //logError("Link '%s' specified as parent for end effector '%s' is not known to the URDF", parent, ename);
            continue;
        }
        if (parent_group) {
            e.parent_group_ = std::string(parent_group); trim(e.parent_group_);
        }
        end_effectors_.push_back(e);
    }
}

void srdf_advr::Model::loadLinkSphereApproximations(const urdf::ModelInterface &urdf_model, tinyxml2::XMLElement *robot_xml) {
    for (tinyxml2::XMLElement *cslink_xml = robot_xml->FirstChildElement("link_sphere_approximation"); cslink_xml; cslink_xml = cslink_xml->NextSiblingElement("link_sphere_approximation")) {
        int non_0_radius_sphere_cnt = 0;
        const char *link_name = cslink_xml->Attribute("link");
        if (!link_name) {
            std::cerr << "Name of link is not specified in link_collision_spheres" << std::endl;
            //logError("Name of link is not specified in link_collision_spheres");
            continue;
        }

        LinkSpheres link_spheres;
        link_spheres.link_ = trim(std::string(link_name));
        if (!urdf_model.getLink(link_spheres.link_)) {
            std::cerr << "Link is not known to URDF." << std::endl;
            //logError("Link '%s' is not known to URDF.", link_name);
            continue;
        }

        // get the spheres for this link
        int cnt = 0;
        for (tinyxml2::XMLElement *sphere_xml = cslink_xml->FirstChildElement("sphere"); sphere_xml; sphere_xml = sphere_xml->NextSiblingElement("sphere"), cnt++) {
            const char *s_center = sphere_xml->Attribute("center");
            const char *s_r = sphere_xml->Attribute("radius");
            if (!s_center || !s_r) {
                std::cerr << "Link collision sphere for link does not have both center and radius." << std::endl;
                //logError("Link collision sphere %d for link '%s' does not have both center and radius.", cnt, link_name);
                continue;
            }

            Sphere sphere;
            try{
                std::stringstream center(s_center);
                center.exceptions(std::stringstream::failbit | std::stringstream::badbit);
                center >> sphere.center_x_ >> sphere.center_y_ >> sphere.center_z_;
                sphere.radius_ = std::stod(s_r);
            }catch (std::stringstream::failure &e) {
                std::cerr << "Link collision sphere for link  has bad center attribute value." << std::endl;
                //logError("Link collision sphere %d for link '%s' has bad center attribute value.", cnt, link_name);
                continue;
            }catch (std::invalid_argument &e) {
                std::cerr << "Link collision sphere for link has bad radius attribute value." << std::endl;
                //logError("Link collision sphere %d for link '%s' has bad radius attribute value.", cnt, link_name);
                continue;
            }catch (std::out_of_range &e) {
                std::cerr << "Link collision sphere for link has bad radius attribute value." << std::endl;
                //logError("Link collision sphere %d for link '%s' has bad radius attribute value.", cnt, link_name);
                continue;
            }

            // ignore radius==0 spheres unless there is only 1 of them
            //
            // NOTE:
            //  - If a link has no sphere_approximation then one will be generated
            //     automatically containing a single sphere which encloses the entire
            //     collision geometry.  Internally this is represented by not having
            //     a link_sphere_approximations_ entry for this link.
            //  - If a link has only spheres with radius 0 then it will not be
            //     considered for collision detection.  In this case the internal
            //     representation is a single radius=0 sphere.
            //  - If a link has at least one sphere with radius>0 then those spheres
            //     are used.  Any radius=0 spheres are eliminated.
            if (sphere.radius_ > std::numeric_limits<double>::epsilon()) {
                if (non_0_radius_sphere_cnt == 0)
                    link_spheres.spheres_.clear();
                link_spheres.spheres_.push_back(sphere);
                non_0_radius_sphere_cnt++;
            } else if (non_0_radius_sphere_cnt == 0) {
                sphere.center_x_ = 0.0;
                sphere.center_y_ = 0.0;
                sphere.center_z_ = 0.0;
                sphere.radius_ = 0.0;
                link_spheres.spheres_.clear();
                link_spheres.spheres_.push_back(sphere);
            }
        }

        if (!link_spheres.spheres_.empty())
            link_sphere_approximations_.push_back(link_spheres);
    }
}

void srdf_advr::Model::loadDisabledCollisions(const urdf::ModelInterface &urdf_model, tinyxml2::XMLElement *robot_xml) {
    for (tinyxml2::XMLElement *c_xml = robot_xml->FirstChildElement("disable_collisions"); c_xml; c_xml = c_xml->NextSiblingElement("disable_collisions")) {
        const char *link1 = c_xml->Attribute("link1");
        const char *link2 = c_xml->Attribute("link2");
        if (!link1 || !link2) {
            std::cerr << "A pair of links needs to be specified to disable collisions" << std::endl;
            //logError("A pair of links needs to be specified to disable collisions");
            continue;
        }
        DisabledCollision dc;
        dc.link1_ = trim(std::string(link1));
        dc.link2_ = trim(std::string(link2));
        if (!urdf_model.getLink(dc.link1_)) {
            std::cerr << "Link is not known to URDF. Cannot disable collisons." << std::endl;
            //logWarn("Link '%s' is not known to URDF. Cannot disable collisons.", link1);
            continue;
        }
        if (!urdf_model.getLink(dc.link2_)) {
            std::cerr << "Link is not known to URDF. Cannot disable collisons." << std::endl;
            //logWarn("Link '%s' is not known to URDF. Cannot disable collisons.", link2);
            continue;
        }
        const char *reason = c_xml->Attribute("reason");
        if (reason)
            dc.reason_ = std::string(reason);
        disabled_collisions_.push_back(dc);
    }
}

void srdf_advr::Model::loadPassiveJoints(const urdf::ModelInterface &urdf_model, tinyxml2::XMLElement *robot_xml) {
    for (tinyxml2::XMLElement *c_xml = robot_xml->FirstChildElement("passive_joint"); c_xml; c_xml = c_xml->NextSiblingElement("passive_joint")) {
        const char *name = c_xml->Attribute("name");
        if (!name) {
            std::cerr << "No name specified for passive joint. Ignoring." << std::endl;
            //logError("No name specified for passive joint. Ignoring.");
            continue;
        }
        PassiveJoint pj;
        pj.name_ = trim(std::string(name));

        // see if a virtual joint was marked as passive
        bool vjoint = false;
        for (std::size_t i = 0; !vjoint && i < virtual_joints_.size(); ++i)
            if (virtual_joints_[i].name_ == pj.name_)
                vjoint = true;

        if (!vjoint && !urdf_model.getJoint(pj.name_)) {
            std::cerr << "Joint marked as passive is not known to the URDF. Ignoring." << std::endl;
            //logError("Joint '%s' marked as passive is not known to the URDF. Ignoring.", name);
            continue;
        }
        passive_joints_.push_back(pj);
    }
}

void srdf_advr::Model::loadDisabledJoints(const urdf::ModelInterface &urdf_model, tinyxml2::XMLElement *robot_xml) {
    for (tinyxml2::XMLElement *c_xml = robot_xml->FirstChildElement("disabled_joint"); c_xml; c_xml = c_xml->NextSiblingElement("disabled_joint")) {
        const char *name = c_xml->Attribute("name");
        if (!name) {
            std::cerr << "No name specified for disabled joint. Ignoring." << std::endl;
            //logError("No name specified for disabled joint. Ignoring.");
            continue;
        }
        DisabledJoint dj;
        dj.name_ = trim(std::string(name));

        disabled_joints_.push_back(dj);
    }
}

void srdf_advr::Model::loadRTTGazebo(const urdf::ModelInterface &urdf_model,
                                     tinyxml2::XMLElement *robot_xml) {
    for (tinyxml2::XMLElement *c_xml = robot_xml->FirstChildElement("rtt-gazebo");
         c_xml; c_xml = c_xml->NextSiblingElement("rtt-gazebo")) {
        RTTGazebo temp;
        tinyxml2::XMLElement *hardware_xml = c_xml->FirstChildElement("hardware");
        if (hardware_xml != NULL) {
            Hardware tempHard;
            tempHard.type_ = trim(
                std::string(hardware_xml->Attribute("type")));
            const char *address = hardware_xml->Attribute("address");
            if (!address) {
                std::cerr << "Missing address for hardware in SRDF" << std::endl;
                continue;
            }
            tempHard.address_ = trim(
                std::string(address));
            const char *port = hardware_xml->Attribute("port");
            if (!port) {
                std::cerr << "Missing port for hardware in SRDF" << std::endl;
                continue;
            }
            tempHard.portNo_ = atoi(port);
            temp.hardware_info_ = tempHard;
        }
        for (tinyxml2::XMLElement *ct_xml = c_xml->FirstChildElement("controller");
             ct_xml; ct_xml = ct_xml->NextSiblingElement("controller")) {
            Controller tempCt;
            tempCt.type_ = trim(
                std::string(ct_xml->Attribute("type")));

            for (tinyxml2::XMLElement *g_xml = ct_xml->FirstChildElement("gains");
                 g_xml; g_xml = g_xml->NextSiblingElement("gains")) {
                Gains tempg;
                if (tempCt.type_ == std::string("JointPositionCtrl")) {
                    tempg.D_ = atof(g_xml->Attribute("D"));
                    tempg.I_ = atof(g_xml->Attribute("I"));
                    tempg.P_ = atof(g_xml->Attribute("P"));
                } else {
                    tempg.D_ = atof(g_xml->Attribute("damping"));
                    //tempg.I_=atof(g_xml->Attribute("I"));
                    tempg.P_ = atof(g_xml->Attribute("stiffness"));
                }
                //tempg.reference_=trim(std::string(g_xml->Attribute("reference")));
                tempCt.gain_params_map_.insert(
                    std::pair<std::string, Gains>(
                        trim(
                            std::string(
                                g_xml->Attribute("reference"))),
                        tempg));
            }
            temp.controllers_.push_back(tempCt);
        }
        rtt_gazebo_.insert(
            std::pair<std::string, RTTGazebo>(
                trim(
                    std::string(c_xml->Attribute("reference"))),
                temp));
    }
}

bool srdf_advr::Model::initXml(const urdf::ModelInterface &urdf_model, tinyxml2::XMLElement *robot_xml) {
    clear();
    if (!robot_xml || std::string(robot_xml->Name()) != "robot") {
        std::cerr << "Could not find the 'robot' element in the xml file" << std::endl;
        //logError("Could not find the 'robot' element in the xml file");
        return false;
    }

    // get the robot name
    const char *name = robot_xml->Attribute("name");
    if (!name)
        std::cerr << "No name given for the robot." << std::endl;
    //logError("No name given for the robot.");
    else {
        name_ = std::string(name); trim(name_);
        if (name_ != urdf_model.getName())
            std::cerr << "Semantic description is not specified for the same robot as the URDF" << std::endl;
        //logError("Semantic description is not specified for the same robot as the URDF");
    }

    loadVirtualJoints(urdf_model, robot_xml);
    loadGroups(urdf_model, robot_xml);
    loadGroupStates(urdf_model, robot_xml);
    loadEndEffectors(urdf_model, robot_xml);
    loadLinkSphereApproximations(urdf_model, robot_xml);
    loadDisabledCollisions(urdf_model, robot_xml);
    loadPassiveJoints(urdf_model, robot_xml);
    loadDisabledJoints(urdf_model, robot_xml);
    loadRTTGazebo(urdf_model, robot_xml);

    return true;
}

bool srdf_advr::Model::initXml(const urdf::ModelInterface &urdf_model, tinyxml2::XMLDocument *xml) {
    tinyxml2::XMLElement *robot_xml = xml ? xml->FirstChildElement("robot") : NULL;
    if (!robot_xml) {
        std::cerr << "Could not find the 'robot' element in the xml file" << std::endl;
        //logError("Could not find the 'robot' element in the xml file");
        return false;
    }
    return initXml(urdf_model, robot_xml);
}

bool srdf_advr::Model::initFile(const urdf::ModelInterface &urdf_model, const std::string &filename) {
    // get the entire file
    std::string xml_string;
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open()) {
        while (xml_file.good()) {
            std::string line;
            std::getline(xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
        return initString(urdf_model, xml_string);
    } else {
        std::cerr << "Could not open file for parsing." << std::endl;
        //logError("Could not open file [%s] for parsing.", filename.c_str());
        return false;
    }
}

bool srdf_advr::Model::initString(const urdf::ModelInterface &urdf_model, const std::string &xmlstring) {
    tinyxml2::XMLDocument xml_doc;
    xml_doc.Parse(xmlstring.c_str());
    return initXml(urdf_model, &xml_doc);
}

void srdf_advr::Model::clear() {
    name_ = "";
    groups_.clear();
    group_states_.clear();
    virtual_joints_.clear();
    end_effectors_.clear();
    link_sphere_approximations_.clear();
    disabled_collisions_.clear();
    passive_joints_.clear();
    disabled_joints_.clear();
    rtt_gazebo_.clear();
}

std::vector<std::pair<std::string, std::string> > srdf_advr::Model::getDisabledCollisions() const {
    std::vector<std::pair<std::string, std::string> > result;
    for (std::size_t i = 0; i < disabled_collisions_.size(); ++i)
        result.push_back(std::make_pair(disabled_collisions_[i].link1_, disabled_collisions_[i].link2_));
    return result;
}
