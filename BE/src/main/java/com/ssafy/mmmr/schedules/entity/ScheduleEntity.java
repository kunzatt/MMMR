package com.ssafy.mmmr.schedules.entity;

import java.time.LocalDateTime;

import org.springframework.data.annotation.CreatedDate;
import org.springframework.data.annotation.LastModifiedDate;
import org.springframework.data.jpa.domain.support.AuditingEntityListener;

import com.ssafy.mmmr.profiles.entity.ProfileEntity;

import jakarta.persistence.Column;
import jakarta.persistence.Entity;
import jakarta.persistence.EntityListeners;
import jakarta.persistence.GeneratedValue;
import jakarta.persistence.GenerationType;
import jakarta.persistence.Id;
import jakarta.persistence.JoinColumn;
import jakarta.persistence.ManyToOne;
import jakarta.persistence.Table;
import lombok.AccessLevel;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@EntityListeners(AuditingEntityListener.class)
@Table(name = "schedules")
public class ScheduleEntity {

	@Id
	@GeneratedValue(strategy = GenerationType.AUTO)
	private Long id;

	@ManyToOne
	@JoinColumn(name = "profile_id", nullable = false)
	private ProfileEntity profile;

	@Column(name = "title", length = 50, nullable = false)
	private String title;

	@Column(name = "start_date", nullable = false)
	private LocalDateTime startDate;

	@Column(name = "end_date", nullable = false)
	private LocalDateTime endDate;

	@CreatedDate
	@Column(name = "created_at", updatable = false)
	private LocalDateTime createdAt;

	@LastModifiedDate
	@Column(name = "updated_at")
	private LocalDateTime updatedAt;

	@Column(name = "deleted", nullable = false)
	private boolean deleted = false;

	@Builder
	public ScheduleEntity(ProfileEntity profile, String title, LocalDateTime startDate, LocalDateTime endDate) {
		this.profile = profile;
		this.title = title;
		this.startDate = startDate;
		this.endDate = endDate;
	}

	public void update(String title, LocalDateTime startDate, LocalDateTime endDate) {
		this.title = title;
		this.startDate = startDate;
		this.endDate = endDate;
	}

	public void delete() {
		this.deleted = true;
	}
}