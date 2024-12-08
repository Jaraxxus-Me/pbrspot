import xml.etree.ElementTree as ET

def update_relationships_after_deletion(root, link_to_delete):
    """
    Updates the parent-child relationships in the URDF after deleting a link.
    
    Parameters:
        root (ElementTree.Element): Root of the URDF XML tree.
        link_to_delete (str): Name of the link to be deleted.
    """
    # Find the parent of the link_to_delete
    parent_link = None
    for joint in root.findall("joint"):
        child_tag = joint.find("child")
        if child_tag is not None and child_tag.attrib.get("link") == link_to_delete:
            parent_tag = joint.find("parent")
            parent_link = parent_tag.attrib.get("link")
            break

    if not parent_link:
        print(f"Link {link_to_delete} has no parent or joint relationship to update.")
        return

    print(f"Link {link_to_delete} is being removed. Its parent is {parent_link}. Updating children...")

    # Update children of the deleted link
    for joint in root.findall("joint"):
        parent_tag = joint.find("parent")
        if parent_tag is not None and parent_tag.attrib.get("link") == link_to_delete:
            # Re-assign the parent to the deleted link's parent
            parent_tag.set("link", parent_link)
            print(f"Updated joint {joint.attrib.get('name')} to have parent {parent_link}.")

def add_links_to_urdf(input_urdf, output_urdf, new_links, link_to_delete=None):
    """
    Edits a URDF file to add new links and joints, and optionally delete a link.
    
    Parameters:
        input_urdf (str): Path to the input URDF file.
        output_urdf (str): Path to save the modified URDF file.
        new_links (list of dict): List of new links with their relationships.
            Each dict should have:
                - name (str): Name of the new link.
                - parent (str): Name of the parent link.
                - xyz (list): Translation of the new link relative to the parent.
                - rpy (list): Rotation of the new link relative to the parent.
        link_to_delete (str): Name of the link to delete from the URDF.
    """
    # Parse the URDF file
    tree = ET.parse(input_urdf)
    root = tree.getroot()

    # Optional: Delete a link and update relationships
    if link_to_delete:
        found = False
        for link in root.findall("link"):
            if link.attrib.get("name") == link_to_delete:
                root.remove(link)
                print(f"Deleted link: {link_to_delete}")
                found = True
                break
        if not found:
            print(f"Link {link_to_delete} not found.")
        else:
            # Update relationships after deletion
            update_relationships_after_deletion(root, link_to_delete)

        # Remove the joint associated with the deleted link
        for joint in root.findall("joint"):
            child_tag = joint.find("child")
            if child_tag is not None and child_tag.attrib.get("link") == link_to_delete:
                root.remove(joint)
                print(f"Deleted joint associated with link: {link_to_delete}")
                break

    # Add new links and joints
    for link in new_links:
        link_element = ET.Element("link", name=link["name"])
        inertial = ET.SubElement(link_element, "inertial")
        origin = ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        mass = ET.SubElement(inertial, "mass", value="0.1")
        inertia = ET.SubElement(inertial, "inertia", 
                                ixx="0.01", ixy="0", ixz="0", 
                                iyy="0.01", iyz="0", 
                                izz="0.01")
        root.append(link_element)

        joint_element = ET.Element("joint", name=f"{link['name']}_joint", type="fixed")
        parent = ET.SubElement(joint_element, "parent", link=link["parent"])
        child = ET.SubElement(joint_element, "child", link=link["name"])
        origin = ET.SubElement(joint_element, "origin", 
                               xyz=" ".join(map(str, link["xyz"])), 
                               rpy=" ".join(map(str, link["rpy"])))
        root.append(joint_element)

    # Write the modified URDF
    tree.write(output_urdf, xml_declaration=True, encoding="utf-8")
    print(f"Modified URDF saved as {output_urdf}")

# Example Usage
input_urdf = "pbrspot/models/spot_description/spot.urdf"
output_urdf = "new_spot.urdf"

new_links = [
    {"name": "hand_frame", "parent": "arm_link_wr1", "xyz": [0.19557, 0.0, 0.0], "rpy": [0, 0, 0]},
]

add_links_to_urdf(input_urdf, output_urdf, new_links)
