import React, { useState } from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from '../contexts/AuthContext';
import styles from './AuthButton.module.css';

export default function AuthButton(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const { user, isAuthenticated, signOut } = useAuth();

  if (!isAuthenticated) {
    return (
      <div className={styles.authButtonContainer}>
        <Link to="/auth" className={styles.authButton}>
          Sign In
        </Link>
      </div>
    );
  }

  return (
    <div className={styles.userMenuContainer}>
      <button
        className={styles.userButton}
        onClick={() => setIsOpen(!isOpen)}
        title={user?.name}
      >
        <img src={user?.image} alt={user?.name} className={styles.avatar} />
        <span>Logged In</span>
      </button>
      {isOpen && (
        <div className={styles.dropdown}>
          <div className={styles.userInfo}>
            <img src={user?.image} alt={user?.name} className={styles.avatarLarge} />
            <div>
              <p className={styles.userName}>{user?.name}</p>
              <p className={styles.userEmail}>{user?.email}</p>
            </div>
          </div>
          <button
            className={styles.signOutButton}
            onClick={() => {
              signOut();
              setIsOpen(false);
            }}
          >
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
}
