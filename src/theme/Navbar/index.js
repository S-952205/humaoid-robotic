import React from 'react';
import NavbarLayout from '@theme/Navbar/Layout';
import NavbarContent from '@theme/Navbar/Content';

// Basic navbar override to maintain Docusaurus theme structure
const CustomNavbar = (props) => {
  return (
    <NavbarLayout>
      <NavbarContent {...props}>
        {/* Render the default children (existing navbar items) */}
        {props.children}
      </NavbarContent>
    </NavbarLayout>
  );
};

export default CustomNavbar;