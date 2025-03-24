package com.ssafy.mmmr.profiles.repository;

import java.util.List;
import java.util.Optional;

import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import com.ssafy.mmmr.profiles.entity.ProfileEntity;

@Repository
public interface ProfileRepository extends JpaRepository<ProfileEntity, Long> {

	boolean existsByNicknameAndDeletedFalse(String nickname);

	List<ProfileEntity> findByAccountIdAndDeletedFalse(Long accountId);

	Optional<ProfileEntity> findByIdAndDeletedFalse(Long id);

	Optional<ProfileEntity> findByIdAndAccountIdAndDeletedFalse(Long id, Long accountId);



}
